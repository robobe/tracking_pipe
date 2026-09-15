# Real GStreamer Pipeline and Missing Components

`tracker_pipeline` is the process that creates this pipeline, receives metadata from the two `appsink`s, runs the independent 50 Hz estimator timer, and publishes fused bbox output.

```mermaid
flowchart LR
    CAM[v4l2src] --> CONVERT[Camera caps and tee]
    CONVERT --> NQ[queue]
    NQ --> NCONV[videoconvert BGR]
    NCONV --> NANO[rknnnanotrack]
    NANO --> NSINK[appsink nano_metadata]

    CONVERT --> YQ[leaky queue]
    YQ --> YRATE[videorate 3 Hz]
    YRATE --> YCONV[videoconvert RGB]
    YCONV --> YOLO[rknnyolov8]
    YOLO --> YSINK[appsink yolo_metadata]

    NSINK --> MISSING1[Missing: tracker_pipeline metadata adapter]
    YSINK --> MISSING1
    MISSING1 --> MISSING2[Missing: bbox_estimator_core at 50 Hz]
    MISSING2 --> OUT[Missing: fused bbox publisher]
    MISSING2 --> RETRACK[Set live Nano roi property]

    CONVERT --> VQ[leaky queue]
    VQ --> VRATE[videorate stream FPS]
    VRATE --> VSCALE[videoscale stream size]
    VSCALE --> ENC[mpph264enc]
    ENC --> RTP[rtph264pay and udpsink]
```

## Pipeline branches

The application builds equivalent GStreamer syntax below. Values in `${...}` are deployment configuration, not shell defaults.

```text
v4l2src device=${CAMERA_DEVICE}
  ! video/x-raw,format=NV12,width=${CAMERA_WIDTH},height=${CAMERA_HEIGHT},framerate=30/1
  ! tee name=camera

camera.
  ! queue max-size-buffers=2 leaky=downstream
  ! videoconvert ! video/x-raw,format=BGR
  ! rknnnanotrack enabled=true roi=${INITIAL_ROI} models-dir=${NANO_MODELS_DIR}
  ! appsink name=nano_metadata emit-signals=true sync=false

camera.
  ! queue max-size-buffers=1 leaky=downstream
  ! videorate drop-only=true ! video/x-raw,framerate=${YOLO_FPS}/1
  ! videoconvert ! video/x-raw,format=RGB
  ! rknnyolov8 model=${YOLO_MODEL}
  ! appsink name=yolo_metadata emit-signals=true sync=false

camera.
  ! queue max-size-buffers=2 leaky=downstream
  ! videorate drop-only=true
  ! videoscale ! videoconvert
  ! video/x-raw,format=I420,width=${STREAM_WIDTH},height=${STREAM_HEIGHT},framerate=${STREAM_FPS}/1
  ! mpph264enc bps=${STREAM_BITRATE_BPS} gop=${STREAM_FPS} header-mode=each-idr
  ! h264parse ! rtph264pay pt=96 config-interval=1
  ! udpsink host=${GROUND_STATION_HOST} port=${VIDEO_PORT} sync=false async=false
```

The input PTS reaches each branch with its source buffer. `tracker_pipeline` must extract `GstVideoRegionOfInterestMeta` and `GST_BUFFER_PTS` in each appsink callback, then enqueue a normalized measurement for `bbox_estimator_core`. Appsink callback arrival time is logged but never used as the measurement timestamp.

## Missing implementation status

| Item | Status | What to implement |
| --- | --- | --- |
| `rknnnanotrack` / `rknnnanotracker12` | Existing in `gst_rknn` | Reuse; setting its live `roi` property requests template reset on the next frame |
| `rknnyolov8` | Existing in `gst_rknn` | Reuse; select the target `yolo8` ROI in the application |
| `roi2udp` | Existing in `gst_rknn` | Optional debug metadata path only |
| `tracker_pipeline` | **Missing** | Application that owns pipeline lifecycle, appsink callbacks, PTS normalization, Nano ROI writes, and bbox transport |
| `bbox_estimator_core` | **Missing** | Pure deterministic 50 Hz startup/prediction/history-correction/re-track engine |
| Fused bbox publisher | **Missing** | Small transport adapter selected after the consumer protocol is chosen; it receives only `EstimatedBBox` from the core |

Do not implement `bbox_estimator_core` as a GStreamer transform: transforms are driven by arriving video buffers at 30 Hz, whereas this component must publish independently at 50 Hz and combine delayed metadata from two branches.

## Build order

1. Implement `bbox_estimator_core` with JSON Lines deterministic replay inputs.
2. Implement `tracker_pipeline` with synthetic appsink-like input first, then replace it with the live branches above.
3. Add the selected fused bbox publisher transport.
4. Add the live Nano `roi` reset after delayed YOLO correction is passing replay tests.
5. Enable the video branch last; it must remain optional to tracking.
