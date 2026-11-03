# PG2L100H_RK3568-2026

基于紫光同创 PG2L100H 与 RK3568 的多路视频辅助驾驶工程。FPGA 完成视频采集、缓存、拼接、PCIe 传输及车道线/斑马线特征处理；RK3568 完成模型推理、结果叠加和 Qt 交互；PC 提供以太网视频输入。

## 工程目录

| 目录 | 内容 |
| --- | --- |
| `FPGA/` | Pango 工程、RTL、IP、约束、实现脚本及当前实现数据 |
| `RK3568/` | PCIe 驱动、Qt 主程序、算法、配置及两份 RKNN 模型 |
| `PC/ethernet_video/` | Windows 桌面采集和 UDP 视频发送程序 |

代码包包含主程序构建与运行所需的工程源码、配置和模型。工具链、操作系统、厂商 SDK 和开发板配套内核构建环境由使用者准备。

## 硬件与环境

- FPGA：PG2L100H，FBG676，速度等级 -6；Pango Design Suite 2023.2-SP3。
- ARM：RK3568，Linux/aarch64；驱动须与目标板内核配置和版本匹配。
- RK 应用：GCC/G++、Make、Qt5Gui、libdrm、OpenSSL、RKNN Runtime、RGA、OpenCV core/imgproc 开发文件。
- RK 显示：HDMI 显示器，支持逐行 1920×1080@60；FPGA 与 RK 通过 PCIe 连接。
- PC：Windows、Python 3、NumPy、Pillow；以太网视频输入使用千兆网口。

RK 启动脚本使用已有的 Ubuntu 用户态目录 `/opt/rk-native/ubuntu`，其中包含 aarch64 loader、Qt 平台插件及 Noto CJK 字体。使用其他兼容目录时设置 `RK_NATIVE_ROOT`。启动前准备该运行环境；具体路径见 `RK3568/scripts/run_dashboard_native.sh`。

## FPGA 构建

用 PDS 打开 `FPGA/video_dma_hdmi.pds`，确认器件、板卡接线与 `FPGA/constraints/` 一致。在 `FPGA/` 工程目录的 Tcl 控制台运行：

```tcl
source impl.tcl
```

顶层为 `video_dma_hdmi_top`。实现完成后，通过 PDS 下载生成的位流。

当前实现配置、资源占用和时序数据见 [当前实现报告](FPGA/TIMING_REPORT.md)。

## RK3568 构建

在 Linux/aarch64 开发环境中，从仓库根目录执行。以下变量分别指向 RKNN/RGA SDK 的目录，以及包含 OpenCV `include/` 和 `lib/` 的目录：

```sh
make -C RK3568 main \
    RKNN_CFLAGS="-I$RKNN_SDK/rknn/include" \
    RKNN_LIBS="-L$RKNN_SDK/rknn/lib -lrknnrt" \
    RGA_CFLAGS="-I$RGA_SDK/include" \
    RGA_LIBS="-L$RGA_SDK/lib -lrga" \
    LIGHT_CV_CFLAGS="-I$OPENCV_SDK/include" \
    LIGHT_CV_LIBS="-L$OPENCV_SDK/lib -lopencv_imgproc -lopencv_core"
```

默认 `make` 与 `make all` 同样构建主程序。产物位于 `build/rk3568-main/`，包含 `video_dma_dashboard`、FPGA 接口工具和 RGB 配对工具。变更 SDK 或编译配置时，使用新的 `MAIN_BUILD_DIR`。

驱动单独构建：`KERNEL_BUILD` 指向目标内核已准备的构建目录，包含对应配置、生成头文件及 `Module.symvers`；交叉构建时另传 `CROSS_COMPILE`。

```sh
make -C RK3568 modules KDIR="$KERNEL_BUILD" ARCH=arm64
```

驱动产物为 `build/rk3568/driver/video_dma.ko`。将编译产物和仓库传到 RK 板，首次加载匹配的驱动，并检查接口：

```sh
insmod /absolute/path/video_dma.ko
./build/rk3568-main/video_dma_fpga_probe /dev/video_dma_0002:21:00.0
```

已有兼容模块时使用现有模块，不重复加载。设备路径以目标板实际枚举结果为准。

## RK3568 启动

连接有效的视频输入与 RK HDMI 显示器，设置板端可见的运行库目录：

```sh
export VDMA_RUNTIME_LIBRARY_PATH="$RKNN_RUNTIME_LIB:$RGA_RUNTIME_LIB:$OPENCV_RUNTIME_LIB"
sh RK3568/scripts/run_main_dashboard.sh \
    --device /dev/video_dma_0002:21:00.0 \
    --mouse /dev/input/by-id/your-mouse-event-mouse
```

三个运行库变量分别指向含 RKNN、RGA 和 OpenCV `.so` 的目录。鼠标路径按 `ls /dev/input/by-id/` 的实际结果填写；不使用鼠标时可省略该参数。脚本管理 Weston 的停止和恢复；Ctrl+C 正常退出，追加 `--seconds 15` 可限时运行。

默认读取 `RK3568/config/yolo.json` 及包内模型。使用其他产物位置时，设置绝对路径变量 `VDMA_DASHBOARD_BINARY`；配置位置可通过 `VDMA_YOLO_CONFIG` 指定。KEY0 由 FPGA 切换双摄、HDMI、Ethernet 和 Quad 画面，当前识别流程使用 HDMI 来源。

## PC 视频发送

在 Windows 安装依赖：

```powershell
python -m pip install numpy Pillow
```

连接 FPGA 千兆网口，将 PC 对应网卡配置为 `192.168.1.10/24`。FPGA 地址为 `192.168.1.100`、MAC 为 `02-00-00-00-00-01`、UDP 端口为 `5000`。首次在管理员 PowerShell 中配置当前会话的静态邻居：

```powershell
.\PC\ethernet_video\start_sender.ps1 -ConfigureNeighbor -CheckOnly -Bind 192.168.1.10
```

随后在普通 PowerShell 中启动：

```powershell
.\PC\ethernet_video\start_sender.ps1 -Bind 192.168.1.10
```

支持 `-Source desktop`、`synthetic`、`pattern`；`-Python` 指定解释器，`-Fps` 设置目标帧率，`-Frames` 设置发送帧数，Ctrl+C 停止。协议使用 1920×1080 RGB444，每帧 3240 个数据包。

## 模型与第三方代码

- 通用模型：YOLOv8n COCO80，RK3568 INT8；输入 RGB NHWC 640×384，视频内容缩放为 640×360、上下各补12行。SHA-256：`30b795e6b152a79c0ef022e799febca816e9a64798c1df096931e804bccc0d3e`。
- 交通灯模型：Autoware YOLOX，RK3568 INT8；输入 416×416 BGR，九组输出。SHA-256：`6486f45fa0ff4cca0385b46f9c95b482f23498c72ca54ee30bd91c25a17f14df`。来源、模型卡、标签及授权声明保留在 `RK3568/models/traffic_light_specialist/`。
- cJSON 的源码及 MIT 许可证保留在 `RK3568/vendor/cjson/`；Pango IP 保留原始声明。各第三方组件按各自授权使用。
- `adas_signal_test`、`adas_lane_test`、`adas_pedestrian_test` 是交互决策功能模块，属于主程序运行代码。
