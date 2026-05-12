"""Generate multi-colour QR test cards for the qr_nav detector.

For each command in ENCODED_COMMANDS, renders one card per (FG, BG) colour
pair in COLOR_PAIRS, so the operator can sweep through them in front of
the OAK-D camera and see where the detector starts dropping codes.

Why this is useful: the detector in
    src/qr_nav/qr_detector_node.py
converts BGR -> grayscale (`cv2.COLOR_BGR2GRAY`) before
`detectAndDecodeMulti`, so the literal colours never reach the QR decoder.
What survives is the luminance contrast between modules and background:

    Y = 0.299 R + 0.587 G + 0.114 B   (Rec. 601 / cv2 grayscale)

CLAHE rescues a lot of low-contrast cases, and the adaptive-threshold
fallback in the detector helps further, but below ~ΔY = 70 things usually
break. The default colour set sweeps from high contrast (ΔY ~ 200) down
to a deliberately weak case (ΔY ~ 65) so you can locate the boundary on
the actual robot, not just on paper.

Outputs (under repo/qr_cards/color_test/):
    <COMMAND>_color_<NN>_<slug>.png   one A4-portrait card per (cmd, pair)
    color_test_<COMMAND>.pdf          per-command deck (print what you need)
    color_test_all.pdf                every card, one per page (commands x pairs)

Usage (from repo root, using the qrgen venv that already has the deps):
    ./.venv-qrgen/bin/python3 repo/src/utils/generate_color_qr_cards.py

Or with the system Python after `pip install qrcode[pil] pillow`:
    python3 repo/src/utils/generate_color_qr_cards.py

Runs on the laptop only. Does not import ROS.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import qrcode
from qrcode.constants import ERROR_CORRECT_H
from PIL import Image, ImageDraw, ImageFont

# Pillow uses lazy plugin registration; without init() the multi-page PDF
# writer (which embeds frames as JPEG) fails with KeyError: 'JPEG'.
Image.init()


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
OUTPUT_DIR = REPO_ROOT / "qr_cards" / "color_test"

# Commands to render a colour sweep for. Order here drives the
# combined-PDF page order. Defaults cover the four navigation primitives
# the operator needs to see actually fire on the bot — GO arms the FSM
# out of IDLE, U_TURN / TURN_LEFT / TURN_RIGHT exercise the timed-turn
# branch of the control loop.
ENCODED_COMMANDS: list[str] = ["GO", "U_TURN", "TURN_LEFT", "TURN_RIGHT"]

# A4 portrait at 150 DPI -> 1240 x 1754 px (same canvas as the main cards).
A4_PX = (1240, 1754)
QR_SIDE_PX = 720           # ~12 cm at 150 DPI
LABEL_HEIGHT_PX = 220
MARGIN_PX = 80
CARD_BG = "white"          # outer card stays white; only the QR tile colours change
CARD_FG = "black"


@dataclass(frozen=True)
class ColorPair:
    """One QR colour combination for the test sweep."""
    name: str           # human-readable, used in the card label
    fg: str             # QR module colour (hex)
    bg: str             # QR background colour (hex)
    expected: str       # "should work" / "stress" — printed on the card

    @property
    def slug(self) -> str:
        """Filesystem-safe slug for the PNG filename."""
        return self.name.lower().replace(" ", "_").replace("/", "_")


# Sweep from high luminance contrast (Δ ~ 200) down to a deliberate
# stress case (Δ ~ 65). Hex codes are Material-design-ish swatches that
# print and photograph well under indoor fluorescent / OAK-D auto-exposure.
COLOR_PAIRS: list[ColorPair] = [
    ColorPair("Navy on White",            "#1A237E", "#FFFFFF", "should work"),
    ColorPair("Black on Yellow",          "#000000", "#FDD835", "should work"),
    ColorPair("Dark Purple on Cream",     "#4A148C", "#FFF8E1", "should work"),
    ColorPair("Dark Green on Light Pink", "#1B5E20", "#FCE4EC", "should work"),
    ColorPair("Dark Red on Light Cyan",   "#B71C1C", "#E0F7FA", "should work"),
    ColorPair("Dark Blue on Light Green", "#0D47A1", "#C8E6C9", "should work"),
    ColorPair("Mid Red on Dark Blue",     "#E53935", "#1A237E", "stress — likely fails"),
]


def hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    """Parse a #RRGGBB hex string."""
    h = hex_color.lstrip("#")
    if len(h) != 6:
        raise ValueError(f"Expected 6-digit hex colour, got {hex_color!r}")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def luminance(hex_color: str) -> float:
    """Rec. 601 / cv2 grayscale luminance, the channel the detector sees."""
    r, g, b = hex_to_rgb(hex_color)
    return 0.299 * r + 0.587 * g + 0.114 * b


def delta_y(pair: ColorPair) -> float:
    """Absolute luminance gap between FG and BG (what survives BGR->GRAY)."""
    return abs(luminance(pair.bg) - luminance(pair.fg))


def load_label_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """Try a few common system fonts; fall back to PIL default if none work."""
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        "Arial.ttf",
        "DejaVuSans-Bold.ttf",
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    print("Warning: no TrueType font found; falling back to PIL default.",
          file=sys.stderr)
    return ImageFont.load_default()


def render_qr(content: str, fg: str, bg: str) -> Image.Image:
    """Render the QR code at QR_SIDE_PX with arbitrary fg/bg colours.

    Uses ERROR_CORRECT_H (~30%) and a border=4 quiet zone so the BG colour
    actually shows around the modules — that quiet zone is part of what
    the detector keys off, so changing its colour is a real part of the
    test, not just decoration.
    """
    qr = qrcode.QRCode(
        version=None,
        error_correction=ERROR_CORRECT_H,
        box_size=20,
        border=4,
    )
    qr.add_data(content)
    qr.make(fit=True)
    img = qr.make_image(fill_color=fg, back_color=bg).convert("RGB")
    return img.resize((QR_SIDE_PX, QR_SIDE_PX), Image.NEAREST)


def render_card(
    command: str,
    pair_index: int,
    pair_total: int,
    pair: ColorPair,
) -> Image.Image:
    """Render one A4 portrait card for a (command, colour-pair) combo."""
    card = Image.new("RGB", A4_PX, CARD_BG)
    qr_img = render_qr(command, pair.fg, pair.bg)

    qr_x = (A4_PX[0] - QR_SIDE_PX) // 2
    qr_y = MARGIN_PX + 260
    card.paste(qr_img, (qr_x, qr_y))

    draw = ImageDraw.Draw(card)

    # Header — what the operator should compare against the camera output.
    header_font = load_label_font(56)
    header_text = f"qr_nav colour test  -  pair {pair_index}/{pair_total}"
    hbbox = draw.textbbox((0, 0), header_text, font=header_font)
    hx = (A4_PX[0] - (hbbox[2] - hbbox[0])) // 2
    draw.text((hx, MARGIN_PX), header_text, fill=CARD_FG, font=header_font)

    # Subheader: encoded command, big so the operator can see what should
    # appear on /qr_command if detection succeeds.
    cmd_font = load_label_font(110)
    cmd_text = f"encodes: {command}"
    cbbox = draw.textbbox((0, 0), cmd_text, font=cmd_font)
    cx = (A4_PX[0] - (cbbox[2] - cbbox[0])) // 2
    draw.text((cx, MARGIN_PX + 90), cmd_text, fill=CARD_FG, font=cmd_font)

    # Colour-pair info block under the QR.
    info_font = load_label_font(48)
    info_lines = [
        pair.name,
        f"FG {pair.fg}   BG {pair.bg}   delta Y = {delta_y(pair):.0f}",
        f"({pair.expected})",
    ]
    line_y = qr_y + QR_SIDE_PX + 50
    for line in info_lines:
        bbox = draw.textbbox((0, 0), line, font=info_font)
        lx = (A4_PX[0] - (bbox[2] - bbox[0])) // 2
        draw.text((lx, line_y), line, fill=CARD_FG, font=info_font)
        line_y += (bbox[3] - bbox[1]) + 18

    foot_font = load_label_font(36)
    foot_text = "ROSbot 3 PRO  -  Project A  -  print at 100% scale"
    fbbox = draw.textbbox((0, 0), foot_text, font=foot_font)
    fx = (A4_PX[0] - (fbbox[2] - fbbox[0])) // 2
    fy = A4_PX[1] - MARGIN_PX - (fbbox[3] - fbbox[1])
    draw.text((fx, fy), foot_text, fill=CARD_FG, font=foot_font)

    return card


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Wipe the legacy single-command PDF if it's still around from an earlier
    # run — the new layout uses per-command file names so an old name lingering
    # would just be confusing.
    legacy_pdf = OUTPUT_DIR / "color_test.pdf"
    if legacy_pdf.exists():
        legacy_pdf.unlink()
        print(f"Removed legacy {legacy_pdf.relative_to(REPO_ROOT)}")

    total_cards = len(ENCODED_COMMANDS) * len(COLOR_PAIRS)
    print(
        f"Generating {total_cards} cards "
        f"({len(ENCODED_COMMANDS)} commands x {len(COLOR_PAIRS)} colour pairs) "
        f"into {OUTPUT_DIR.relative_to(REPO_ROOT)}"
    )
    print()

    all_cards: list[Image.Image] = []  # for the combined PDF, in command order

    for command in ENCODED_COMMANDS:
        print(f"  [{command}]")
        per_cmd_cards: list[Image.Image] = []
        for idx, pair in enumerate(COLOR_PAIRS, start=1):
            card = render_card(command, idx, len(COLOR_PAIRS), pair)
            out_png = OUTPUT_DIR / f"{command}_color_{idx:02d}_{pair.slug}.png"
            card.save(out_png, dpi=(150, 150))
            per_cmd_cards.append(card)
            all_cards.append(card)
            print(
                f"    - pair {idx}/{len(COLOR_PAIRS)}  {pair.name:<28}  "
                f"delta_Y={delta_y(pair):6.1f}  -> "
                f"{out_png.relative_to(REPO_ROOT)}"
            )

        per_cmd_pdf = OUTPUT_DIR / f"color_test_{command}.pdf"
        per_cmd_cards[0].save(
            per_cmd_pdf,
            save_all=True,
            append_images=per_cmd_cards[1:],
            resolution=150.0,
        )
        print(
            f"    => {per_cmd_pdf.relative_to(REPO_ROOT)}  "
            f"({len(per_cmd_cards)} pages)"
        )
        print()

    combined_pdf = OUTPUT_DIR / "color_test_all.pdf"
    all_cards[0].save(
        combined_pdf,
        save_all=True,
        append_images=all_cards[1:],
        resolution=150.0,
    )
    print(
        f"  => {combined_pdf.relative_to(REPO_ROOT)}  "
        f"({len(all_cards)} pages, every command x pair)"
    )
    print()
    print("Done. Print at 100% scale (no 'fit to page') to keep the QR ~12 cm.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
