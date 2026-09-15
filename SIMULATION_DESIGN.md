# Deterministic Estimator Simulation

`bbox_sim` is the fastest development and debug loop for the tracking pipeline. It runs the same C++ `bbox_estimator_core` intended for `tracker_pipeline`, but has no camera, GStreamer, RKNN, network, thread, or wall-clock dependency.

## Build and run

```bash
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/bbox_sim tests/fixtures/delayed_yolo.jsonl --trace
```

Compare a run to its golden output:

```bash
./build/bbox_sim tests/fixtures/delayed_yolo.jsonl \
  --expected tests/fixtures/delayed_yolo.expected.jsonl
```

## Input format

The simulator reads JSON Lines. A `measurement` becomes visible at `arrival_ns` but is fused at its original `capture_pts_ns`.

```json
{"type":"measurement","arrival_ns":60000000,"capture_pts_ns":0,"source":"yolo8","bbox_xywh_px":[12,20,20,10],"confidence":0.9,"class_id":0}
{"type":"tick","time_ns":60000000}
```

Supported sources are `nanotrack` and `yolo8`. Ticks are explicit 50 Hz times (normally 20 ms apart). Before each tick, the runner submits every measurement whose arrival time is no later than that tick. Same-capture-time measurements are ordered Nano first, then YOLO, then file sequence.

## Output and replay behavior

One JSON record is emitted for every tick. It contains the estimated bbox, centre velocity, startup/tracking status, validity, decisions for newly arrived measurements, and any requested Nano re-track ROI.

The estimator keeps a bounded 500 ms history. A late YOLO result inside that window rebuilds estimator state from its capture time to the current tick using saved normalized measurements; no tracker is executed again. Input older than the history window is rejected as `outside_history`.

The initial estimator is deliberately a small constant-velocity, confidence-weighted filter. Its parameters are in `tracking_pipe::Config`; tune them only from replay recordings.

## Fixtures

| Fixture | Verifies |
| --- | --- |
| `startup` | `INITIALIZING` after one measurement and valid prediction after the second |
| `delayed_yolo` | A late YOLO correction changes future output only |
| `bad_yolo` | Association gate rejects an implausible detection |
| `retrack` | High-confidence YOLO plus weak, disagreeing Nano requests one reset ROI |

To reproduce a field issue later, have `tracker_pipeline` write the normalized appsink records in this same format. Replaying that file with the same configuration produces the same estimates regardless of camera or NPU timing.
