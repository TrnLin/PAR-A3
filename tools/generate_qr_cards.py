#!/usr/bin/env python3
"""Generate printable QR-code command cards for the qr_nav package.

Reads the canonical command list directly from
    src/qr_nav/qr_nav/qr_detector_node.py (VALID_COMMANDS)
via a tiny AST parse, so the printed cards can never silently drift out of
sync with the strings the detector actually accepts.

Outputs (under repo/qr_cards/):
    <COMMAND>.png   - one A4-portrait card per command, ~12 cm QR side,
                      with a large human-readable label below the code.
    all_commands.pdf - all cards concatenated, one per A4 page, ready to
                      print double-sided or single-sided.

Usage:
    pip install qrcode[pil] pillow
    python tools/generate_qr_cards.py

This script runs on the laptop. It does not import ROS and never touches
the robot.
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import qrcode
from qrcode.constants import ERROR_CORRECT_H
from PIL import Image, ImageDraw, ImageFont

# Pillow uses lazy plugin registration; without this, Image.SAVE is empty
# and the multi-page PDF writer (which embeds frames as JPEG) fails with
# KeyError: 'JPEG'.
Image.init()


REPO_ROOT = Path(__file__).resolve().parent.parent
DETECTOR_SOURCE = REPO_ROOT / "src" / "qr_nav" / "qr_nav" / "qr_detector_node.py"
OUTPUT_DIR = REPO_ROOT / "qr_cards"

# A4 portrait at 150 DPI -> 1240 x 1754 px. Plenty for printing.
A4_PX = (1240, 1754)
QR_SIDE_PX = 720           # ~12 cm at 150 DPI
LABEL_HEIGHT_PX = 220
MARGIN_PX = 80
BG_COLOR = "white"
FG_COLOR = "black"


def load_valid_commands(source_path: Path) -> list[str]:
    """Extract the VALID_COMMANDS set literal from the detector source.

    Using AST instead of import-execing the module avoids pulling in rclpy
    / cv2 just to read a constant.
    """
    tree = ast.parse(source_path.read_text())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        targets = [t.id for t in node.targets if isinstance(t, ast.Name)]
        if "VALID_COMMANDS" not in targets:
            continue
        if not isinstance(node.value, ast.Set):
            raise RuntimeError(
                f"VALID_COMMANDS in {source_path} is not a set literal "
                f"(got {type(node.value).__name__}); update this script."
            )
        commands = []
        for elt in node.value.elts:
            if not isinstance(elt, ast.Constant) or not isinstance(elt.value, str):
                raise RuntimeError(
                    f"Non-string element in VALID_COMMANDS: {ast.dump(elt)}"
                )
            commands.append(elt.value)
        # Sort for stable, predictable output order.
        return sorted(commands)
    raise RuntimeError(f"Could not find VALID_COMMANDS in {source_path}")


def load_label_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    """Try a few common system fonts; fall back to PIL default if none work."""
    candidates = [
        # macOS
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
        # Linux (Ubuntu/Debian)
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        # Common cross-platform
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


def render_qr(command: str) -> Image.Image:
    """Render the QR code for a command at QR_SIDE_PX x QR_SIDE_PX."""
    qr = qrcode.QRCode(
        version=None,
        error_correction=ERROR_CORRECT_H,  # ~30% recovery
        box_size=20,
        border=2,
    )
    qr.add_data(command)
    qr.make(fit=True)
    img = qr.make_image(fill_color=FG_COLOR, back_color=BG_COLOR).convert("RGB")
    return img.resize((QR_SIDE_PX, QR_SIDE_PX), Image.NEAREST)


def render_card(command: str) -> Image.Image:
    """Render one A4 portrait card: QR centred horizontally, label below."""
    card = Image.new("RGB", A4_PX, BG_COLOR)
    qr_img = render_qr(command)

    qr_x = (A4_PX[0] - QR_SIDE_PX) // 2
    qr_y = MARGIN_PX + 240   # leave room for a header
    card.paste(qr_img, (qr_x, qr_y))

    draw = ImageDraw.Draw(card)

    header_font = load_label_font(64)
    header_text = "qr_nav command"
    hbbox = draw.textbbox((0, 0), header_text, font=header_font)
    hx = (A4_PX[0] - (hbbox[2] - hbbox[0])) // 2
    draw.text((hx, MARGIN_PX), header_text, fill=FG_COLOR, font=header_font)

    label_font = load_label_font(140)
    lbbox = draw.textbbox((0, 0), command, font=label_font)
    lx = (A4_PX[0] - (lbbox[2] - lbbox[0])) // 2
    ly = qr_y + QR_SIDE_PX + 60
    draw.text((lx, ly), command, fill=FG_COLOR, font=label_font)

    foot_font = load_label_font(40)
    foot_text = "ROSbot 3 PRO  -  Project A"
    fbbox = draw.textbbox((0, 0), foot_text, font=foot_font)
    fx = (A4_PX[0] - (fbbox[2] - fbbox[0])) // 2
    fy = A4_PX[1] - MARGIN_PX - (fbbox[3] - fbbox[1])
    draw.text((fx, fy), foot_text, fill=FG_COLOR, font=foot_font)

    return card


def main() -> int:
    commands = load_valid_commands(DETECTOR_SOURCE)
    if not commands:
        print("No commands found in VALID_COMMANDS.", file=sys.stderr)
        return 1

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    print(f"Generating {len(commands)} cards into {OUTPUT_DIR}")

    cards: list[Image.Image] = []
    for cmd in commands:
        card = render_card(cmd)
        out_png = OUTPUT_DIR / f"{cmd}.png"
        card.save(out_png, dpi=(150, 150))
        cards.append(card)
        print(f"  - {out_png.relative_to(REPO_ROOT)}")

    pdf_path = OUTPUT_DIR / "all_commands.pdf"
    cards[0].save(
        pdf_path,
        save_all=True,
        append_images=cards[1:],
        resolution=150.0,
    )
    print(f"  - {pdf_path.relative_to(REPO_ROOT)}  ({len(cards)} pages)")
    print("Done. Print at 100% scale (no 'fit to page') to keep the QR ~12 cm.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
