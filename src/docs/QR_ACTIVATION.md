# QR Activation: How a QR Code in the Frame Triggers a Command

There is **no manually-defined region of interest (ROI)** in the camera frame.
The detector scans the **entire image**, and OpenCV's QR decoder returns the
QR's pixel coordinates itself. We then use those coordinates — specifically the
bounding-box **area** — as the gate that decides whether to fire a command.

All logic lives in `repo/src/qr_nav/qr_detector_node.py`.

---

## 1. Find QR(s) anywhere in the frame

OpenCV does the localization for us. It scans the full grayscale / CLAHE-
enhanced frame and returns the 4 corner points of every QR it finds:

```python
retval, decoded_info, points, straight_qrcode = (
    self.qr_detector.detectAndDecodeMulti(image)
)
```

`points` is an `Nx4x2` array of `(x, y)` pixel coordinates — that is the QR's
location in the picture frame.

---

## 2. Convert those corners into a bbox area

```python
bbox_points = np.asarray(points[i], dtype=np.float32)
bbox_area = cv2.contourArea(bbox_points)
```

The area of the polygon formed by those 4 corners is our proxy for **distance**:
a closer QR occupies more pixels.

> Note: `cv2.contourArea` on OpenCV 4.6 (ROS 2 Jazzy default) requires `CV_32F`
> or `CV_32S`. `np.float64` (the default for `.astype(float)`) raises
> `(-215:Assertion failed) depth == CV_32F || CV_32S`. We explicitly cast to
> `np.float32` and wrap in `try/except` so a single malformed detection becomes
> a warning instead of crashing the node.

---

## 3. Gate the command on area + valid command set

```python
if content in VALID_COMMANDS and bbox_area >= self.min_bbox_area:
    valid_detections.append((content, bbox_area))
```

Two conditions must hold to "activate" the command:

- decoded text is in `VALID_COMMANDS`
  (`STOP`, `GO`, `TURN_LEFT`, `TURN_RIGHT`, `SPEED_UP`, `SPEED_DOWN`, `U_TURN`)
- `bbox_area >= min_bbox_area`
  (default **500 px²**, set in `repo/src/config/qr_params.yaml`)

---

## 4. If multiple QRs are visible, pick the closest one

```python
best_command, best_area = max(valid_detections, key=lambda x: x[1])
```

Largest bbox → closest QR → wins.

---

## 5. Cooldown prevents re-firing

```python
last_time = self.last_command_time.get(best_command, 0.0)
if now - last_time < self.cooldown_sec:
    return
```

The same command cannot re-trigger within `cooldown_sec` (1.0 s), so a QR
sitting in the frame for many frames only publishes once.

---

## Activation rule (summary)

A command is published to `/qr_command` when **all** of the following hold for
a detected QR:

| Check | Source |
|---|---|
| Decoded successfully by `cv2.QRCodeDetector` | corners returned in `points[i]` |
| Text ∈ `VALID_COMMANDS` | `qr_detector_node.py:21-29` |
| `contourArea(points[i]) >= min_bbox_area` | distance gate (px²) |
| Largest bbox among valid detections | "closest wins" tiebreaker |
| Not within `cooldown_sec` of previous fire of same command | de-bounce |

---

## What about the "coordinate" in the frame?

It is **implicit**. We never define an ROI in the picture frame — we just use
the QR's own corner pixel positions (returned by OpenCV) and threshold on
**bbox area**.

If you ever want to restrict activation to, for example, only the center of the
frame (ignore QRs at the edges), you would add a check on the centroid of
`points[i]` against an image-pixel ROI **before** appending to
`valid_detections`. Something like:

```python
cx = float(np.mean(bbox_points[:, 0]))
cy = float(np.mean(bbox_points[:, 1]))
H, W = image.shape[:2]
in_roi = (
    0.25 * W <= cx <= 0.75 * W and
    0.25 * H <= cy <= 0.75 * H
)
if content in VALID_COMMANDS and bbox_area >= self.min_bbox_area and in_roi:
    valid_detections.append((content, bbox_area))
```

This would gate activation on both **size** (close enough) and **position**
(centered in frame).

---

## Tunable parameters

Defined in `repo/src/config/qr_params.yaml` and re-readable as ROS 2 params:

| Parameter | Default | Effect |
|---|---|---|
| `min_bbox_area` | `500` px² | minimum apparent QR size to count as "close enough" |
| `detection_rate_hz` | `15.0` Hz | how often the image callback actually runs detection |
| `cooldown_sec` (hardcoded) | `1.0` s | de-bounce window per command |
| `rgb_image_topic` | `/oak/rgb/image_raw` | camera topic |
| `image_qos_reliability` | `reliable` | must match publisher QoS or zero frames arrive |
