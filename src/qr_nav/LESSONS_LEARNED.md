# qr_nav — Bring-up Lessons Learned

Forensic record of every issue hit while taking `qr_nav` from "code compiles on a Mac" to "robot drives on the floor". Written in chronological order — roughly the deployment pipeline order — so it doubles as a debugging story.

This is the source material for the report's *Evaluation & Discussion* section: it shows that the failure modes were systematic (mostly distro-version mismatches between the development and target environments) rather than random.

## Target platform (the recurring villain)

Most issues here come from the gap between the assumptions baked into off-the-shelf ROS 2 examples (typically ROS 2 Humble, OpenCV 4.5, plain `Twist`, generic camera namespaces) and what the **Husarion ROSbot 3 PRO actually ships**:

| Item            | What examples assume       | What the ROSbot 3 PRO actually has |
| --------------- | -------------------------- | ---------------------------------- |
| OS              | Ubuntu 22.04               | Ubuntu 24.04                       |
| ROS distro      | Humble                     | **Jazzy**                          |
| OpenCV          | 4.5.x                      | **4.6.0**                          |
| Camera topic    | `/camera/camera/...`       | `/oak/rgb/...`                     |
| Camera QoS      | BEST_EFFORT (typical)      | **RELIABLE**                       |
| `/cmd_vel` type | `geometry_msgs/Twist`      | **`geometry_msgs/TwistStamped`**   |
| Camera FPS      | ~30                        | ~10 (Pi 5 CPU-bound)               |

Every gotcha below is one cell of that table biting back.

---

## Issue 1 — Wrong camera topic name

**Symptom**

```
husarion@husarion:~$ ros2 topic hz /camera/camera/color/image_raw
WARNING: topic [/camera/camera/color/image_raw] does not appear to be published yet
```

The detector node started without errors but `/qr_command` never fired.

**Root cause**

The `qr_detector_node` defaulted to the depthai-ros standard topic `/camera/camera/color/image_raw`. Husarion's bring-up uses a different launch that namespaces the OAK-D under `/oak/...`. The actual image topic is `/oak/rgb/image_raw`.

**Diagnosis**

```bash
ros2 topic list | grep oak
# /oak/rgb/image_raw   <-- here it is
```

(The first `grep` we ran filtered for `camera|cmd_vel`, which only matched `/oak/rgb/camera_info` because of the substring "camera". Misleading. Always `grep oak` on Husarion.)

**Fix**

Updated [`config/qr_params.yaml`](config/qr_params.yaml) `rgb_image_topic` and the in-code default in [`qr_detector_node.py`](qr_nav/qr_detector_node.py) to `/oak/rgb/image_raw`. Documented in a comment so the next person doesn't redo the discovery.

**Takeaway**

Never hard-code OEM topic names. Verify with `ros2 topic list` on the actual robot and parameterise the topic path so it can be changed via YAML without touching code.

---

## Issue 2 — QoS mismatch (silent receiver failure)

**Symptom**

After fixing the topic name, the detector node started cleanly but `ros2 topic echo /qr_command` showed nothing. No errors, no warnings, no logs. The node just sat there.

**Root cause**

The subscriber was created with `ReliabilityPolicy.BEST_EFFORT` (default for camera streams). Husarion's OAK-D publisher uses `RELIABLE`. In ROS 2, a `BEST_EFFORT` subscriber against a `RELIABLE` publisher does NOT receive any messages — the QoS contract is rejected and the matching just silently fails.

**Diagnosis**

```bash
ros2 topic info /oak/rgb/image_raw -v
# QoS profile:
#   Reliability: RELIABLE     <-- key line
#   Durability:  VOLATILE
```

**Fix**

Made the QoS reliability a parameter (`image_qos_reliability`, default `"reliable"`), wired through to the actual subscriber in [`qr_detector_node.py`](qr_nav/qr_detector_node.py). Startup banner now prints which QoS was applied so it's verifiable at a glance:

```
QR Detector Node started — topic: /oak/rgb/image_raw, ..., qos: RELIABLE
```

**Takeaway**

ROS 2 QoS mismatches are the #1 silent-failure trap. **Always** verify the publisher's QoS with `ros2 topic info -v` and either match it exactly or use a compatible profile. Print the chosen QoS at startup so you can spot misconfiguration without digging through logs.

---

## Issue 3 — Wrong message type on /cmd_vel (Jazzy convention)

**Symptom**

After fixing topic + QoS, the detector decoded cards perfectly. But when we fired up the full launch, the robot wouldn't move. Worse: `ros2 run teleop_twist_keyboard teleop_twist_keyboard` also did nothing — the keyboard worked, but the wheels didn't.

The web UI (Husarion's foxglove dashboard) drove the robot fine. So motors were OK and the network was OK.

**Root cause**

ROS 2 Jazzy + recent Husarion images switched `/cmd_vel` from `geometry_msgs/Twist` to `geometry_msgs/TwistStamped`. Our nodes (and the default `teleop_twist_keyboard`) publish plain `Twist`. The base controller subscribes to `TwistStamped`. Type mismatch → silent drop, same pattern as the QoS issue.

**Diagnosis**

```bash
ros2 topic info /cmd_vel
# Type: geometry_msgs/msg/TwistStamped     <-- not Twist
```

We also confirmed who else publishes by listing publishers:

```bash
ros2 topic list | grep -i cmd | xargs -I{} sh -c 'ros2 topic info {} -v 2>&1 | grep "Node name"'
# foxglove_bridge   <-- web UI, correctly publishing TwistStamped
# joy2twist         <-- joystick, correctly publishing TwistStamped
```

**Fix**

Updated both [`command_interpreter_node.py`](qr_nav/command_interpreter_node.py) and [`data_logger_node.py`](qr_nav/data_logger_node.py) to import and use `TwistStamped`:

```python
from geometry_msgs.msg import TwistStamped

self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

msg = TwistStamped()
msg.header.stamp = self.get_clock().now().to_msg()
msg.header.frame_id = 'base_link'
msg.twist.linear.x = linear_x
msg.twist.angular.z = angular_z
```

Subscriber/logger likewise reads `msg.twist.linear.x` instead of `msg.linear.x`. For teleop, the workaround is the `--stamped` parameter:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

**Takeaway**

ROS 2's "stamped" message variants are creeping into mainstream stacks. When porting code from Humble to Jazzy or to a newer hardware platform, **check the message type of every interaction topic with `ros2 topic info`** before trusting that the historical type still applies. Symptom is identical to QoS mismatch — silent drop with no error.

---

## Issue 4 — OpenCV 4.6 dtype strictness in `cv2.contourArea`

**Symptom**

After fixing the message type, the detector banner came up correctly, frames were arriving, and a printed STOP card was held in front of the camera. The detector then crashed with:

```
cv2.error: OpenCV(4.6.0) ./modules/imgproc/src/shapedescr.cpp:315:
error: (-215:Assertion failed) npoints >= 0 && (depth == CV_32F || depth == CV_32S)
in function 'contourArea'
[ros2run]: Process exited with failure 1
```

Crucially, this only fired on the *fallback* (adaptive-threshold) detection path, after CLAHE failed to find the QR. It would have crashed on the primary path too if a QR ever decoded there with the offending dtype.

**Root cause**

The bbox area calculation was:

```python
bbox_area = cv2.contourArea(bbox_points.astype(float))
```

In NumPy, `astype(float)` defaults to `float64`, which OpenCV calls `CV_64F`. **OpenCV 4.6.0** (which ROS 2 Jazzy ships) requires the points to be `CV_32F` (`np.float32`) or `CV_32S` (`np.int32`). Newer OpenCV (4.13 on the dev machine) is more permissive and accepts `float64` silently, which is why the bug never surfaced during local testing.

**Diagnosis**

The traceback pointed at the exact line, and the assertion message named the required dtype constants. Cross-referencing with `cv2.__version__` on the robot (`4.6.0`) vs the dev machine (`4.13.0`) confirmed the version-specific strictness.

**Fix**

Changed the cast to `np.float32` and added a `try/except` guard so a single malformed bbox produces a logged warning instead of crashing the node mid-drive (which would strand the robot in whatever motion state it had):

```python
import numpy as np
...
try:
    bbox_points = np.asarray(points[i], dtype=np.float32)
    bbox_area = cv2.contourArea(bbox_points)
except Exception as e:
    self.get_logger().warn(
        f'Skipping detection {i!r}={content!r}: bbox area calc failed ({e})'
    )
    continue
```

**Takeaway**

Two lessons here.

1. *Library version must match the target environment.* Running OpenCV 4.13 locally and 4.6 on the robot is asking for surprises. Either pin the dev machine to the same version (annoying), or add a small CI check that imports key modules on the target distro.
2. *Robot-side errors must never be unhandled.* A perception node throwing an uncaught exception kills the whole node — and on a moving robot, that means the last `cmd_vel` keeps applying until the watchdog catches up. **All per-frame work must be wrapped in `try/except` so transient input weirdness becomes a warning, not a crash.**

---

## Issue 5 — Pillow lazy plugin loading (laptop-side, card generation)

**Symptom**

The QR-card generator script wrote 7 PNGs, then crashed when concatenating them into a multi-page PDF:

```
File "/.../PIL/PdfImagePlugin.py", line 151, in _write_image
    Image.SAVE["JPEG"](im, op, filename)
KeyError: 'JPEG'
```

`features.check('jpg')` returned `True`, yet `Image.SAVE` was empty (`[]`).

**Root cause**

Pillow uses **lazy plugin registration**. Codec handlers are only inserted into `Image.SAVE` after the first time an image is opened or saved using each format. Pillow's PDF writer embeds frames as JPEG and looks up `Image.SAVE["JPEG"]` directly, which had never been registered because the script generated images from scratch and only saved PNGs first.

**Fix**

One line at the top of [`tools/generate_qr_cards.py`](../../../tools/generate_qr_cards.py):

```python
Image.init()
```

This forces Pillow to scan and register all bundled plugins immediately.

**Takeaway**

Library lazy initialisation is the most annoying class of "the function exists but isn't usable yet" bug. When a feature exists per `features.check()` but throws `KeyError` on use, suspect lazy loading and look for an `init()`/`preinit()` API.

---

## Issue 6 — Camera frame rate underperformance (not blocking, but worth flagging)

**Symptom**

```
ros2 topic hz /oak/rgb/image_raw
average rate: 8.141
average rate: 9.000
average rate: 10.272   <-- slowly climbed to ~10 Hz
```

We expected ~30 Hz (the OAK-D Pro's native rate).

**Root cause**

The Pi 5 is CPU-bound running the depthai-ros driver, foxglove_bridge, and the rest of Husarion's bring-up simultaneously. The OAK-D's USB stream throttles to whatever the host can drain.

**Mitigation**

No fix needed for our use case. At 0.08 m/s cruise speed, 10 Hz detection means the robot moves ~8 mm per frame — well within the QR detector's tolerance. The card-detect distance and bbox-area gating keep this from being a problem.

If CPU becomes contentious later (e.g. when running with reactive_nav stacks), drop `detection_rate_hz` in [`qr_params.yaml`](config/qr_params.yaml) from 15 → 10 so the detector doesn't busy-spin trying to process frames the camera never produced.

**Takeaway**

Always measure with `ros2 topic hz` instead of trusting the data sheet. Real frame rates on resource-constrained ARM hosts are usually a fraction of advertised. Tune detection rate to match measured input rate, not assumed hardware capability.

---

## Cross-cutting themes

Patterns that emerged across these issues:

1. **Distro-version drift is the #1 source of bugs.** Topic names, message types, OpenCV strictness, QoS defaults — the gap between Humble assumptions and Jazzy reality bit us four times in this single bring-up.
2. **Silent failures dominate.** Three of six issues (camera topic, QoS, message type) presented as "the node is running but nothing happens" with zero log output. The robot was usable for diagnosing only because we had `ros2 topic info`/`echo`/`hz` to externally observe the graph.
3. **Print your assumptions.** The detector now prints its actual topic, rate, min_bbox_area, and QoS at startup. Before that, "is the QoS reliable?" required digging through code. Tiny banner, big debugging time saved.
4. **Wrap every per-frame callback in `try/except`.** Robotics code that raises uncaught exceptions strands physical hardware. Exceptions in a perception loop should be downgraded to warnings + skip-this-frame, not crashes.
5. **A round-trip test is worth a thousand sanity checks.** The QR-card generator decodes its own outputs with the same `cv2.QRCodeDetector` class the robot uses, before we ever print. That test is what gives us confidence the cards will actually decode on the robot — and would catch any future change in `VALID_COMMANDS` immediately.

---

## Summary table

| #   | Issue                              | Root cause                          | Where it lived                        | Fix                                                           |
| --- | ---------------------------------- | ----------------------------------- | ------------------------------------- | ------------------------------------------------------------- |
| 1   | Camera topic mismatch              | OEM topic namespace differs         | `qr_params.yaml`, `qr_detector_node` | Default to `/oak/rgb/image_raw`, parameterise                  |
| 2   | QoS mismatch (silent drop)         | BEST_EFFORT vs RELIABLE             | `qr_detector_node` subscriber         | Parameterise QoS reliability, default `reliable`              |
| 3   | `/cmd_vel` type mismatch (silent)  | Jazzy uses `TwistStamped`           | `command_interpreter`, `data_logger` | Switch both to `TwistStamped`                                 |
| 4   | OpenCV 4.6 dtype crash             | `float64` vs required `float32`     | `qr_detector_node._detect_qr_codes`  | `np.float32` cast + try/except guard                          |
| 5   | Pillow `KeyError: 'JPEG'`           | Lazy plugin registration            | `tools/generate_qr_cards.py`          | `Image.init()` at module top                                  |
| 6   | Camera at 10 Hz instead of 30      | Pi 5 CPU-bound                      | (not in our code)                     | None needed; align `detection_rate_hz` to real rate if pressed |
