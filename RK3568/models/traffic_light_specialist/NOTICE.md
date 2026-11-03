# Upstream Attribution

The CPU preprocessing and YOLOX decoding adapter in
`RK3568/lib/light_specialist.cpp` derives from Autoware Universe's
`preprocess.cu` and `tensorrt_yolox.cpp`, copyright 2023 TIER IV, Inc.,
licensed under Apache License 2.0. Local changes adapt preprocessing to CPU,
RKNN inference and the frozen-image validation harness.

Upstream commit: `a7b9d4b125e5bf920fe2fa590518f5371b3f715a`.
Original source and license are archived in the project's
`artifacts/validation/traffic_light/2026-11-01/provenance/` directory.
The license is included here as `LICENSE-Apache-2.0.txt`.

The ONNX model is AutowareFoundation traffic_light_fine_detector v3.0,
`tlr_car_ped_yolox_s_batch_1.onnx`. Original SHA256:
`1ad633066a1195006f4709f8fa07800dd65a74a814b3efb4c99bcc5a1a7962f6`.
Preserve the upstream model card and labels alongside the model.
