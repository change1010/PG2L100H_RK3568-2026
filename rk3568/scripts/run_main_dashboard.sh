#!/bin/sh
# Main FPGA road + YOLO + Qt entry; run_display.sh manages Weston.
set -eu
script_dir=$(CDPATH= cd "$(dirname "$0")" && pwd)
root=$(CDPATH= cd "$script_dir/.." && pwd)
if [ -x "$script_dir/video_dma_dashboard" ]; then
    default_binary=$script_dir/video_dma_dashboard
else
    default_binary=$root/../build/rk3568-main/video_dma_dashboard
fi
export VDMA_DASHBOARD_BINARY=${VDMA_DASHBOARD_BINARY:-$default_binary}
config=${VDMA_YOLO_CONFIG:-$root/config/yolo.json}
case "$VDMA_DASHBOARD_BINARY" in /*) ;; *) echo 'Use an absolute binary path' >&2; exit 2 ;; esac
case "$config" in /*) ;; *) echo 'Use an absolute config path' >&2; exit 2 ;; esac
test -x "$VDMA_DASHBOARD_BINARY"
test -r "$config"
test -x "$script_dir/run_dashboard_native.sh"
light_mode=${VDMA_LIGHT_MODE:-resident}
case "$light_mode" in
    resident|legacy) ;;
    *) echo 'VDMA_LIGHT_MODE expects resident or legacy' >&2; exit 2 ;;
esac
unset VDMA_TRIAL_RESIDENT_LIGHT VDMA_TRIAL_MAIN_LIGHT_CPU
case "${VDMA_LIGHT_ENABLED:-1}" in
    1)
        export VDMA_LIGHT_MODEL=${VDMA_LIGHT_MODEL:-$root/models/traffic_light_specialist/traffic_light_yolox_heads_int8_rk3568.rknn}
        case "$VDMA_LIGHT_MODEL" in /*) ;; *) echo 'Use an absolute specialist model path' >&2; exit 2 ;; esac
        test -r "$VDMA_LIGHT_MODEL"
        expected=${VDMA_LIGHT_MODEL_SHA256:-6486f45fa0ff4cca0385b46f9c95b482f23498c72ca54ee30bd91c25a17f14df}
        actual=$(sha256sum "$VDMA_LIGHT_MODEL")
        actual=${actual%% *}
        test "$actual" = "$expected" || { echo 'Specialist model SHA256 mismatch' >&2; exit 1; }
        export VDMA_LIGHT_SEARCH_POLICY=${VDMA_LIGHT_SEARCH_POLICY:-roi-first}
        export VDMA_LIGHT_PREPROCESS=${VDMA_LIGHT_PREPROCESS:-mapped}
        export VDMA_LIGHT_TASK_PERIOD_MS=${VDMA_LIGHT_TASK_PERIOD_MS:-334}
        export VDMA_LIGHT_PROFILE=${VDMA_LIGHT_PROFILE:-1}
        case "$light_mode" in
            resident) export VDMA_LIGHT_RESIDENT=1 VDMA_MAIN_LIGHT_CPU=0 ;;
            legacy) export VDMA_LIGHT_RESIDENT=0 VDMA_MAIN_LIGHT_CPU=1 ;;
        esac
        printf 'MAIN_LIGHT model=%s sha256=%s policy=%s preprocess=%s period_ms=%s profile=%s mode=%s resident=%s main_light_cpu=%s\n' \
            "$VDMA_LIGHT_MODEL" "$actual" "$VDMA_LIGHT_SEARCH_POLICY" "$VDMA_LIGHT_PREPROCESS" \
            "$VDMA_LIGHT_TASK_PERIOD_MS" "$VDMA_LIGHT_PROFILE" "$light_mode" \
            "$VDMA_LIGHT_RESIDENT" "$VDMA_MAIN_LIGHT_CPU" >&2
        ;;
    0)
        unset VDMA_LIGHT_MODEL
        export VDMA_LIGHT_RESIDENT=0 VDMA_MAIN_LIGHT_CPU=1
        ;;
    *) echo 'VDMA_LIGHT_ENABLED expects 0 or 1' >&2; exit 2 ;;
esac
export VDMA_RUNTIME_LIBRARY_PATH=${VDMA_RUNTIME_LIBRARY_PATH:-$root/lib}
export VIDEO_DMA_DISPLAY="$script_dir/run_dashboard_native.sh"
export VDMA_PRESENT_MODE=${VDMA_PRESENT_MODE:-qt-exact}
export VDMA_EXACT_DELAY_MS=${VDMA_EXACT_DELAY_MS:-50}
export VDMA_PRESENT_HISTORY=${VDMA_PRESENT_HISTORY:-1}
export VDMA_YOLO_OSD_MODE=${VDMA_YOLO_OSD_MODE:-hold}
export VDMA_YOLO_COPY_MODE=${VDMA_YOLO_COPY_MODE:-async}
export VDMA_SMALL_RGB_MODE=${VDMA_SMALL_RGB_MODE:-small}
export VDMA_SAME_FRAME_DELAY_MS=${VDMA_SAME_FRAME_DELAY_MS:-0}
export VDMA_CLEAN_VIDEO_DELAY_MS=${VDMA_CLEAN_VIDEO_DELAY_MS:-0}
export VDMA_PRESENT_TEST_WORKLOAD=${VDMA_PRESENT_TEST_WORKLOAD:-full}
export VDMA_ADAS_NOTIFY=${VDMA_ADAS_NOTIFY:-1}
printf 'Main dashboard: binary=%s config=%s present=%s road=FPGA small_rgb=%s\n' \
    "$VDMA_DASHBOARD_BINARY" "$config" "$VDMA_PRESENT_MODE" "$VDMA_SMALL_RGB_MODE" >&2
exec sh "$script_dir/run_display.sh" --recognition-on --yolo-config "$config" "$@"
