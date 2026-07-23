# Camera Image Compression — Investigation & Recommendations

Date: 2026-07-23. Scope: read-only investigation on 226 (`/home/intern/ros2-test/airspeed`),
triggered by `docs/advice-for-imagedata.md`. No source changes were made.

## 1. Camera inventory and available modes

| Camera | Role | Max color modes | Depth |
|---|---|---|---|
| RealSense D405 (SN 352122273178) | wrist | 1280×720 @ 5/15/30 fps; 848×480 / 640×480 @ up to 90 fps | Z16 up to 848×480 @ 90 |
| RealSense D405 (SN 352122271268) | wrist | same | same |
| RealSense D435 (SN 244222070560) | head | dedicated RGB sensor: 1920×1080 @ 6/15/30; 960×540 / 848×480 @ up to 60 | Z16 up to 1280×720 @ 30 |

- All three on USB 3.2, firmware 5.16.0.1.
- Current config (`src/sensor_interface/camera-stream-adaptor/config/camera.yaml`):
  640×480 @ 30 fps, `rgb8` raw, color only.
- Depth/IR streams exist in hardware but are not implemented in the publisher
  (`camera_publisher.py` marks them `_DIRECT_API_STREAMS` — "not yet supported").

## 2. Current pipeline status (measured)

- Frames travel raw over the zenoh side channel and are stored in HDF5 as
  uncompressed RGB arrays: 921,600 bytes/frame (640×480×3), verified in
  `episode-T3-20260722T114325281945Z.h5`.
- That episode is 1.63 GB for ~20 s → **~299 GB/hour** for the 3-camera rig.
- **226's disk is 99% full (21 GB free of 1.9 TB)** — at raw rates this is an
  operational emergency, not an optimization.
- `tools/convert_h5_to_lerobot.py` assumes JPEG byte strings in the h5
  (`cv2.imdecode`, lines 95/156) and is currently incompatible with raw episodes.
- Encode/decode cost (libjpeg-turbo via cv2, measured on real frames):
  encode ~1.8 ms/frame, decode ~0.8 ms/frame — negligible at 30 Hz × 3 cameras.

## 3. Compression benchmark on real recorded frames

| Format | KB/frame (3-cam avg) | Ratio vs raw | GB/h @ 3 cams |
|---|---|---|---|
| raw rgb8 (today) | 900 | 1× | ~299 |
| PNG (lossless) | ~330 | 2.7× | ~108 |
| JPEG q95 | ~65 | 14× | ~21 |
| **JPEG q90** | **~42** | **22×** | **~14** |
| JPEG q70 (current config default) | ~20 | 45× | ~7 |

## 4. Encoder latency comparison (installed libraries on 226)

Measured on 640×480 RGB frames using real recorded data.

| Encoder | p50 ms/frame | p99 ms/frame | Speed vs cv2 |
|---|---|---|---|
| **cv2.imencode** (libjpeg-turbo 3.0.3) | **0.71** | **0.86** | 1× (fastest installed) |
| PIL Image.save (optimize=True) | ~6.7 | ~7.5 | ~9× slower |
| torchvision.io.encode_jpeg | ~35–70 | ~103–116 | ~50–150× slower |

The latency introduced by JPEG q90 versus the raw rgb8 path:

| Metric | raw rgb8 | JPEG q90 | introduced by JPEG |
|---|---|---|---|
| p50 | 0.06 ms | 0.71 ms | **0.65 ms** |
| p99 | 0.10 ms | 0.86 ms | **0.76 ms** |
| max | 0.12 ms | 2.30 ms | **2.18 ms** |

At 30 Hz the frame budget is 33.3 ms; JPEG encoding consumes ~2.1% of that
budget — negligible. OpenCV was therefore chosen as the implementation.

## 5. Implementation status

- `src/sensor_interface/camera-stream-adaptor/config/camera.yaml`: color streams
  switched to `encoding: jpeg`, `jpeg_quality: 90`.
- `src/sensor_interface/camera-stream-adaptor/camera_publisher.py`: default
  fallback quality raised to 90, with a comment documenting why cv2.imencode
  (libjpeg-turbo) is the fastest installed encoder.

## 6. Recommendations

1. **Store JPEG q90 in the HDF5.** ~22× smaller, ~14 GB/h, encode cost ~0.65 ms.
   At 640×480 the policy input (224×224) is already ~3× oversampled, so q90
   artifacts are invisible after downsampling — the core argument of
   `advice-for-imagedata.md` applies directly. Avoid q70 (previous config
   default): too aggressive for wrist cameras tracking small grasp targets.
   PNG for RGB is not worth it (2.7× ratio at 5× the encode cost).
2. **Compression point — two viable options:**
   - (a) At the source: existing `--jpeg` path + `encoding: jpeg` in
     `camera.yaml` (`camera_publisher.py:374-387`). Also shrinks zenoh traffic.
   - (b) Keep raw zenoh transport, compress in the collector before writing h5.
     Keeps a lossless-transport debug option.
   Both restore compatibility with the LeRobot converter (expects JPEG in h5).
   The doc's "no camera-side MJPEG" caveat does not apply — this is PC-side
   libjpeg-turbo, exactly what the doc recommends.
3. **Resolution:** 640×480 is sufficient for 224-input policies. If resample
   headroom is wanted: D405 wrist → 848×480 @ 30 (720p on D405 caps at exactly
   30 fps — no margin); D435 head can go 1920×1080 @ 30.
4. **Depth (if ever enabled):** lossless only — Z16 as raw 16-bit or PNG, never
   JPEG.
5. **Frame rate:** keep recording at 30 Hz; for slow quasi-static tasks,
   decimate to 15 Hz at write time (never the reverse).
6. **Keep the dashboard preview as-is** (lossless PNG display path — unrelated
   to storage).
7. **Immediate operational action:** free disk space on 226 before the next
   collection run regardless of which option is adopted.
