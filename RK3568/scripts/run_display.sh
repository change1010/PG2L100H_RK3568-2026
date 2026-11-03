#!/bin/sh
set -eu
script_dir=$(CDPATH= cd "$(dirname "$0")" && pwd)
display_binary=${VIDEO_DMA_DISPLAY:-$script_dir/video_dma_display}
if [ -z "${VIDEO_DMA_DISPLAY:-}" ] && [ ! -x "$display_binary" ]; then
    display_binary=$script_dir/../../build/rk3568/video_dma_display
fi
cd "$script_dir"
weston_service=/etc/init.d/S49weston
restore_weston=0
display_pid=

finish()
{
    trap - EXIT INT TERM HUP
    if [ -n "$display_pid" ]; then
        kill -TERM "$display_pid" 2>/dev/null || true
        wait "$display_pid" 2>/dev/null || true
    fi
    if [ "$restore_weston" = 1 ] && ! pgrep -x weston >/dev/null; then
        "$weston_service" start </dev/null >/tmp/video_dma_weston_restart.log 2>&1
        echo 'Weston restart requested; log: /tmp/video_dma_weston_restart.log'
    fi
}

trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
if [ ! -x "$display_binary" ]; then
    printf 'Display executable not found: %s\nSet VIDEO_DMA_DISPLAY to an absolute executable path.\n' "$display_binary" >&2
    exit 1
fi
if pgrep -x weston >/dev/null; then
    test -x "$weston_service"
    restore_weston=1
    "$weston_service" stop
    attempts=0
    while pgrep -x weston >/dev/null; do
        attempts=$((attempts + 1))
        if [ "$attempts" -ge 50 ]; then
            echo 'Weston did not stop; display not started.' >&2
            exit 1
        fi
        sleep 0.1
    done
fi
"$display_binary" "$@" &
display_pid=$!
result=0
wait "$display_pid" || result=$?
display_pid=
exit "$result"
