# vision - Raspberry Pi 5 Perception & Pillar Detection

`main.py` - the perception node running on the **Raspberry Pi 5 (8GB)** during the obstacle round. It
processes video streams from the **160° FOV fisheye camera**, identifies red and green obstacle pillars,
computes their color, horizontal angular offset, and bounding box area, and transmits advisory navigation
frames to the STM32 over high-speed UART. In conjunction with the **Slamtec RPLIDAR C1**, it establishes
a robust obstacle mapping and avoidance pipeline. Telemetry is parsed in real time and can be monitored via
a lightweight local diagnostics stream.

## Run it

```
python3 main.py                 # detection + live stream on :8000
python3 main.py --no-stream     # stream off (use for scored runs)
python3 main.py --print         # console output at ~2 Hz
python3 main.py --no-serial     # bench test with no STM32 attached
python3 main.py --calibrate-camera
```

## How it works

1. Grab a frame, crop to the region of interest.
2. Convert to Lab colour space and threshold for red and green using the calibrated ranges in
   `color_config.json` (with sensible fallbacks baked in).
3. Clean up with morphological open/close, take the largest qualifying blob per colour.
4. Report the winner as `V,<colour>,<dx>,<area>`.
5. In parallel, parse the STM32 `#` telemetry and render it on the web page.

## Config files

| File | Contents |
|---|---|
| color_config.json | Lab thresholds + morphology kernel sizes per colour |
| roi_config.json | The detection region of interest |
| camera_config.json | Camera calibration (fisheye) |

## Notes for scored runs

- Run with `--no-stream` to keep the Pi load down.
- WiFi and Bluetooth are disabled on the Pi for competition.
- The link is the same UART the STM32 uses for telemetry - one wire pair, full-duplex.

Vision pipeline authored by Team Blueprint.
