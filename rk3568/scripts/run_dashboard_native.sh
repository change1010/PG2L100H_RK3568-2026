#!/bin/sh
# Host launcher using an existing Ubuntu userspace. No mounts/startup changes.
set -eu
native_root=${RK_NATIVE_ROOT:-/opt/rk-native/ubuntu}
: "${VDMA_DASHBOARD_BINARY:?Set the absolute host path to video_dma_dashboard}"
test -x "$VDMA_DASHBOARD_BINARY"
export QT_QPA_PLATFORM=offscreen
export QT_QPA_PLATFORM_PLUGIN_PATH="$native_root/usr/lib/aarch64-linux-gnu/qt5/plugins/platforms"
export VDMA_FONT_FILE="$native_root/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/qt-dark-runtime}
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
binary_dir=$(dirname "$VDMA_DASHBOARD_BINARY")
exec "$native_root/lib/ld-linux-aarch64.so.1" --library-path "${VDMA_RUNTIME_LIBRARY_PATH:+$VDMA_RUNTIME_LIBRARY_PATH:}$binary_dir:$native_root/lib/aarch64-linux-gnu:$native_root/usr/lib/aarch64-linux-gnu" "$VDMA_DASHBOARD_BINARY" "$@"
