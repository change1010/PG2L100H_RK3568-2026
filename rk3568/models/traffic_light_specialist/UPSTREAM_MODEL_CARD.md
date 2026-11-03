---
license: apache-2.0
pipeline_tag: object-detection
tags:
  - autoware
  - ros2
  - autonomous-driving
  - camera
  - traffic-light
  - yolox
  - tensorrt
  - onnx
---

# YOLOX-s Traffic Light Fine Detector for Autoware (`traffic_light_fine_detector`)

Camera-based traffic light detection models, used by the
[`autoware_traffic_light_fine_detector`](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_fine_detector)
node in [Autoware](https://github.com/autowarefoundation/autoware).

The models are **YOLOX-s** [1] detectors fine-tuned by TIER IV for traffic light detection. Given the full
camera image and the coarse ROIs predicted by the `traffic_light_map_based_detector` node, the fine detector
refines each ROI to a tight bounding box around the traffic light. The models are exported as ONNX; Autoware
builds the TensorRT engine from the ONNX file on first launch.

## Model overview

| | |
| --- | --- |
| Task | 2D traffic light detection (ROI refinement) in camera images |
| Architecture | YOLOX-s |
| Detected classes | `traffic_light`, `pedestrian_traffic_light` (plus a `BACKGROUND` label) |
| Runtime | TensorRT (FP16 by default, FP32 selectable) via the `autoware_traffic_light_fine_detector` ROS 2 node |
| Format | ONNX (Autoware builds the TensorRT engine locally on first launch) |
| License | Apache-2.0 |

## Variants in this repository

The three ONNX files share the same weights and differ only in their static batch size. The TensorRT engine
requires a fixed batch size, so the node pads the ROI list with dummy entries up to the model's batch size
when fewer traffic lights are visible.

| Variant | Batch size | Default in Autoware |
| --- | --- | --- |
| `tlr_car_ped_yolox_s_batch_1.onnx` | 1 | |
| `tlr_car_ped_yolox_s_batch_4.onnx` | 4 | |
| `tlr_car_ped_yolox_s_batch_6.onnx` | 6 | yes (loaded by the package's launch file) |

## Files

| File | Description |
| --- | --- |
| `tlr_car_ped_yolox_s_batch_1.onnx` | YOLOX-s traffic light detector, static batch size 1 |
| `tlr_car_ped_yolox_s_batch_4.onnx` | YOLOX-s traffic light detector, static batch size 4 |
| `tlr_car_ped_yolox_s_batch_6.onnx` | YOLOX-s traffic light detector, static batch size 6 (default) |
| `tlr_labels.txt` | Class labels: `BACKGROUND`, `traffic_light`, `pedestrian_traffic_light` |
| `deploy_metadata.yaml` | Deployment metadata recording the artifact version of this repository |

> **TensorRT engines are not distributed here.** TensorRT engines are specific to the GPU architecture and
> TensorRT version they are built on and are not portable, so Autoware builds them locally from the ONNX files
> on first launch (or via `build_only:=true`).

## Inputs and outputs (as used by the node)

**Inputs**

| Topic | Type | Description |
| --- | --- | --- |
| `~/input/image` | `sensor_msgs/msg/Image` | The full size camera image |
| `~/input/rois` | `tier4_perception_msgs/msg/TrafficLightRoiArray` | Coarse ROIs from the map-based detector |
| `~/expect/rois` | `tier4_perception_msgs/msg/TrafficLightRoiArray` | Offset-free ROIs used to select the best detections |

**Outputs**

| Topic | Type | Description |
| --- | --- | --- |
| `~/output/rois` | `tier4_perception_msgs/msg/TrafficLightRoiArray` | The refined, accurate ROIs |
| `~/debug/exe_time_ms` | `autoware_internal_debug_msgs/msg/Float32Stamped` | Inference time |

If no traffic light is detected inside a coarse ROI, the output ROI's `x_offset`, `y_offset`, `height`, and
`width` are set to `0`. Detections from YOLOX are matched against the `expect/rois` array as a whole set, not
each ROI in isolation. Key node parameters (defaults): `precision: fp16`, `score_thresh: 0.3`,
`nms_thresh: 0.65`, `gpu_id: 0`.

## Usage in Autoware

The node expects these artifacts under `~/autoware_data/ml_models/traffic_light_fine_detector/` and launches
with, e.g.:

```bash
ros2 launch autoware_traffic_light_fine_detector traffic_light_fine_detector.launch.xml \
  data_path:=$HOME/autoware_data/ml_models \
  model_path:=$HOME/autoware_data/ml_models/traffic_light_fine_detector/tlr_car_ped_yolox_s_batch_6.onnx \
  label_path:=$HOME/autoware_data/ml_models/traffic_light_fine_detector/tlr_labels.txt
```

Add `build_only:=true` to build the TensorRT engine from the ONNX as a one-off pre-task.
See the [package README](https://github.com/autowarefoundation/autoware_universe/tree/main/perception/autoware_traffic_light_fine_detector)
for the full parameter reference.

## Training

- Base model: YOLOX-s from the official [YOLOX repository](https://github.com/Megvii-BaseDetection/YOLOX),
  starting from the published [yolox_s.pth](https://github.com/Megvii-BaseDetection/YOLOX/releases/download/0.1.1rc0/yolox_s.pth)
  pretrained weights.
- Fine-tuning: performed by TIER IV on around 17,000 internal images of Japanese traffic lights.
- Further training details (epochs, hyperparameters, evaluation metrics) are not publicly documented.

## Limitations

- Fine-tuned on Japanese traffic lights; accuracy on traffic lights with different shapes, layouts, or
  signaling conventions may drop without additional fine-tuning.
- The node is designed to run downstream of the map-based detector: it refines externally provided ROIs
  rather than detecting traffic lights anywhere in the frame.
- The TensorRT engine has a static batch size; scenes with more simultaneous ROIs than the model's batch size
  are processed in multiple inference passes.

## Provenance

| | |
| --- | --- |
| Original source | `https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v3/` |
| Version tag in this repository | `v3.0` |

## Citation

```bibtex
@article{ge2021yolox,
  title   = {YOLOX: Exceeding YOLO Series in 2021},
  author  = {Ge, Zheng and Liu, Songtao and Wang, Feng and Li, Zeming and Sun, Jian},
  journal = {arXiv preprint arXiv:2107.08430},
  year    = {2021}
}
```

## References

- [1] Ge et al., "YOLOX: Exceeding YOLO Series in 2021", arXiv:2107.08430, 2021.
- YOLOX repository: <https://github.com/Megvii-BaseDetection/YOLOX>
