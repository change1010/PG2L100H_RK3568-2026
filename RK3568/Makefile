RK_ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
.DEFAULT_GOAL := main
.SECONDEXPANSION:

BUILD_DIR ?= $(RK_ROOT)/../build/rk3568
MAIN_BUILD_DIR ?= $(RK_ROOT)/../build/rk3568-main
.PHONY: main
main:
	$(MAKE) -C "$(RK_ROOT)" BUILD_DIR="$(MAIN_BUILD_DIR)" FPGA_ROAD=1 YOLO=1 ROAD_REF=0 dashboard fpga-probe rgb-pair-probe

OUTPUT_ROOT := $(abspath $(BUILD_DIR))
.PHONY: test-presentation
test-presentation: $(OUTPUT_ROOT)/test_presentation
	"$(OUTPUT_ROOT)/test_presentation"
$(OUTPUT_ROOT)/test_presentation: $(RK_ROOT)/tests/test_presentation.cpp $(RK_ROOT)/lib/video_dma_dashboard_history.h $(RK_ROOT)/lib/video_dma_dashboard_osd.h $(RK_ROOT)/include/video_dma_presentation.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" "$<" -o "$@"
.PHONY: test-light-temporal
test-light-temporal: $(OUTPUT_ROOT)/test_light_temporal
	"$(OUTPUT_ROOT)/test_light_temporal"
$(OUTPUT_ROOT)/test_light_temporal: $(RK_ROOT)/tests/test_light_temporal.c $(RK_ROOT)/lib/light_temporal.c $(RK_ROOT)/lib/traffic_light_color.c $(RK_ROOT)/include/light_temporal.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
.PHONY: test-ui-buffers
test-ui-buffers: $(OUTPUT_ROOT)/test_ui_buffers
	"$(OUTPUT_ROOT)/test_ui_buffers"
$(OUTPUT_ROOT)/test_ui_buffers: $(RK_ROOT)/tests/test_ui_buffers.c $(RK_ROOT)/lib/video_dma_ui_buffers.c $(RK_ROOT)/include/video_dma_ui_buffers.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
MODULE_DIR := $(OUTPUT_ROOT)/driver
KDIR ?= /lib/modules/$(shell uname -r)/build
PKG_CONFIG ?= pkg-config
DRM_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags libdrm)
DRM_LIBS ?= $(shell $(PKG_CONFIG) --libs libdrm)
RGA_CFLAGS ?=
RGA_LIBS ?=
RKNN_CFLAGS ?=
RKNN_LIBS ?= -lrknnrt
YOLO ?= 0
FPGA_ROAD ?= 0
ROAD_REF ?= 0
ROAD_REF_ONLY ?= $(ROAD_REF)
ifeq ($(FPGA_ROAD),1)
ifneq ($(ROAD_REF),0)
$(error FPGA_ROAD=1 is incompatible with ROAD_REF)
endif
ROAD_REF_ONLY := 1
DASHBOARD_YOLO_CFLAGS += -DVDMA_HAVE_FPGA_ROAD -DVDMA_ROAD_REF_ONLY
DASHBOARD_ROAD_SOURCES = $(RK_ROOT)/lib/road_zebra_osd.c
DASHBOARD_FPGA_ROAD_SOURCES = $(RK_ROOT)/lib/fpga_road.c $(RK_ROOT)/lib/fpga_road_osd.c
else
DASHBOARD_FPGA_ROAD_SOURCES = $(RK_ROOT)/lib/fpga_road_osd.c
endif
ifeq ($(ROAD_REF_ONLY),1)
ifneq ($(ROAD_REF),1)
ifneq ($(FPGA_ROAD),1)
$(error ROAD_REF_ONLY=1 requires ROAD_REF=1)
endif
endif
DASHBOARD_YOLO_CFLAGS += -DVDMA_ROAD_REF_ONLY
DASHBOARD_ROAD_SOURCES = $(RK_ROOT)/lib/road_zebra_osd.c
else
DASHBOARD_ROAD_SOURCES = $(ZEBRA_SOURCES) $(ROAD_CV_SOURCES)
endif
ROAD_REF_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags opencv4)
ROAD_REF_LIBS ?= $(shell $(PKG_CONFIG) --libs opencv4)
ifeq ($(ROAD_REF),1)
DASHBOARD_YOLO_CFLAGS += -DVDMA_HAVE_ROAD_REF
DASHBOARD_REF_SOURCES = $(RK_ROOT)/lib/road_ref_adas.c $(RK_ROOT)/lib/road_ref_transport.c
DASHBOARD_REF_OBJECTS = $(addprefix $(OUTPUT_ROOT)/dashboard/reference/,$(addsuffix .o,road_ref lane crosswalk))
DASHBOARD_REF_LIBS = $(ROAD_REF_LIBS) -lz
endif
ifeq ($(YOLO),1)
DASHBOARD_YOLO_CFLAGS += -DVDMA_HAVE_YOLO $(RKNN_CFLAGS)
DASHBOARD_YOLO_LIBS = $(RKNN_LIBS) -lcrypto
DASHBOARD_YOLO_SOURCES = $(YOLO_SOURCES)
DASHBOARD_YOLO_SOURCES += $(RK_ROOT)/lib/yolo_adas_frozen.c
DASHBOARD_YOLO_SOURCES += $(RK_ROOT)/lib/light_specialist_color.c
DASHBOARD_REF_OBJECTS += $(OUTPUT_ROOT)/dashboard/lib/light_specialist.o
DASHBOARD_REF_LIBS += $(LIGHT_CV_LIBS)
LIGHT_CV_CFLAGS ?= $(shell $(PKG_CONFIG) --cflags opencv4)
LIGHT_CV_LIBS ?= -lopencv_imgproc -lopencv_core
endif
YOLO_CORE_SOURCES = $(RK_ROOT)/lib/yolo_preprocess.c $(RK_ROOT)/lib/yolo_decode.c $(RK_ROOT)/lib/traffic_light_color.c $(RK_ROOT)/lib/light_temporal.c
YOLO_CORE_HEADERS = $(RK_ROOT)/include/yolo_preprocess.h $(RK_ROOT)/include/yolo_decode.h $(RK_ROOT)/include/traffic_light_color.h $(RK_ROOT)/include/light_temporal.h
YOLO_SOURCES = $(YOLO_CORE_SOURCES) $(RK_ROOT)/lib/yolo_rknn.c $(RK_ROOT)/lib/yolo_adas.c $(RK_ROOT)/lib/yolo_adas_map.c $(RK_ROOT)/lib/yolo_config.c
YOLO_CONFIG_SOURCES = $(RK_ROOT)/lib/yolo_config.c $(RK_ROOT)/vendor/cjson/cJSON.c
YOLO_HEADERS = $(YOLO_CORE_HEADERS) $(RK_ROOT)/include/yolo_rknn.h $(RK_ROOT)/include/yolo_adas.h $(RK_ROOT)/include/yolo_config.h

.PHONY: test-yolo-config
test-yolo-config: $(OUTPUT_ROOT)/test_yolo_config
	"$(OUTPUT_ROOT)/test_yolo_config"
$(OUTPUT_ROOT)/test_yolo_config: $(RK_ROOT)/tests/test_yolo_config.c $(YOLO_CONFIG_SOURCES) $(RK_ROOT)/include/yolo_config.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(UI_LAYOUT_CFLAGS) $(filter %.c,$^) -o "$@" -lcrypto -lm

.PHONY: yolo-live-probe
yolo-live-probe: $(OUTPUT_ROOT)/yolo_live_probe
$(OUTPUT_ROOT)/yolo_live_probe: $(RK_ROOT)/apps/yolo_live_probe.c $$(YOLO_SOURCES) $$(YOLO_HEADERS) $$(SNAPSHOT_SOURCES) $$(SNAPSHOT_HEADERS) $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/vendor/cjson/cJSON.c
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(RKNN_CFLAGS) $(SNAPSHOT_CFLAGS) $(UI_LAYOUT_CFLAGS) $(filter %.c,$^) -o "$@" $(RKNN_LIBS) $(RGA_LIBS) -lcrypto -lm

.PHONY: test-yolo-core yolo-offline-probe
.PHONY: test-yolo-rknn
test-yolo-rknn: $(OUTPUT_ROOT)/test_yolo_rknn
	"$(OUTPUT_ROOT)/test_yolo_rknn"
$(OUTPUT_ROOT)/test_yolo_rknn: $(RK_ROOT)/tests/test_yolo_rknn.c $(RK_ROOT)/lib/yolo_rknn.c $(RK_ROOT)/lib/yolo_decode.c $(RK_ROOT)/include/yolo_rknn.h $(RK_ROOT)/include/yolo_decode.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(RKNN_CFLAGS) $(filter %.c,$^) -o "$@" -lm
.PHONY: test-yolo-decode-parity
test-yolo-decode-parity: $(OUTPUT_ROOT)/test_yolo_decode_parity
	"$(OUTPUT_ROOT)/test_yolo_decode_parity"
$(OUTPUT_ROOT)/test_yolo_decode_parity: $(RK_ROOT)/tests/test_yolo_decode_parity.c $(RK_ROOT)/lib/yolo_decode.c $(RK_ROOT)/include/yolo_decode.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
.PHONY: yolo-batch-probe
yolo-batch-probe: $(OUTPUT_ROOT)/yolo_batch_probe
$(OUTPUT_ROOT)/yolo_batch_probe: $(RK_ROOT)/apps/yolo_batch_probe.c $(RK_ROOT)/lib/yolo_rknn.c $(RK_ROOT)/include/yolo_rknn.h $(YOLO_CORE_SOURCES) $(YOLO_CORE_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(RKNN_CFLAGS) $(filter %.c,$^) -o "$@" $(RKNN_LIBS) -lm
.PHONY: test-yolo-adas
.PHONY: test-light-color
.PHONY: light-color-probe
light-color-probe: $(OUTPUT_ROOT)/light_color_probe
$(OUTPUT_ROOT)/light_color_probe: $(RK_ROOT)/apps/light_color_probe.c $(RK_ROOT)/lib/traffic_light_color.c $(RK_ROOT)/include/traffic_light_color.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
test-light-color: $(OUTPUT_ROOT)/test_light_color
	"$(OUTPUT_ROOT)/test_light_color"
$(OUTPUT_ROOT)/test_light_color: $(RK_ROOT)/tests/test_light_color.c $(RK_ROOT)/lib/traffic_light_color.c $(RK_ROOT)/include/traffic_light_color.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
test-yolo-core: $(OUTPUT_ROOT)/test_yolo_core
	"$(OUTPUT_ROOT)/test_yolo_core"
$(OUTPUT_ROOT)/test_yolo_core: $(RK_ROOT)/tests/test_yolo_core.c $(YOLO_CORE_SOURCES) $(YOLO_CORE_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
yolo-offline-probe: $(OUTPUT_ROOT)/yolo_offline_probe
.PHONY: yolo-adas-offline-probe
yolo-adas-offline-probe: $(OUTPUT_ROOT)/yolo_adas_offline_probe
$(OUTPUT_ROOT)/yolo_adas_offline_probe: $(RK_ROOT)/apps/yolo_adas_offline_probe.c $(RK_ROOT)/lib/yolo_adas.c $(RK_ROOT)/lib/yolo_adas_map.c $(RK_ROOT)/lib/yolo_rknn.c $(RK_ROOT)/lib/adas_result.c $(YOLO_CORE_SOURCES) $(YOLO_CORE_HEADERS) $(RK_ROOT)/include/yolo_adas.h $(RK_ROOT)/include/yolo_rknn.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(RKNN_CFLAGS) $(filter %.c,$^) -o "$@" $(RKNN_LIBS) -lm
test-yolo-adas: $(OUTPUT_ROOT)/test_yolo_adas
	"$(OUTPUT_ROOT)/test_yolo_adas"
$(OUTPUT_ROOT)/test_yolo_adas: $(RK_ROOT)/tests/test_yolo_adas.c $(RK_ROOT)/lib/yolo_adas.c $(RK_ROOT)/lib/yolo_adas_map.c $(RK_ROOT)/lib/adas_result.c $(YOLO_CORE_SOURCES) $(YOLO_CORE_HEADERS) $(RK_ROOT)/include/yolo_adas.h $(RK_ROOT)/include/yolo_rknn.h
$(OUTPUT_ROOT)/test_yolo_adas: $(RK_ROOT)/lib/detection_overlay.c $(RK_ROOT)/include/detection_overlay.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
$(OUTPUT_ROOT)/yolo_offline_probe: $(RK_ROOT)/apps/yolo_offline_probe.c $(RK_ROOT)/lib/yolo_rknn.c $(RK_ROOT)/include/yolo_rknn.h $(YOLO_CORE_SOURCES) $(YOLO_CORE_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(RKNN_CFLAGS) $(filter %.c,$^) -o "$@" $(RKNN_LIBS) -lm
SNAPSHOT_CFLAGS = $(if $(strip $(RGA_CFLAGS)),-DVDMA_HAVE_RGA $(RGA_CFLAGS))
SDK_DRM_ROOT = $(SDK_ROOT)/external/linux-rga/samples/utils/3rdparty/libdrm
SDK_DRM_MODE_HEADERS = $(SDK_ROOT)/external/camera_engine_rkaiq/rkisp_demo/demo/include

.PHONY: all modules display display-sdk control test-feature-validation test-road-features test-road-cv test-road-zebra-dense test-road-zebra-adas road-cv-replay test-road-cv-adas test-metadata test-perception package clean

# Qt paints offscreen; the embedded C loop remains the only video/DRM owner.
ifeq ($(FPGA_ROAD),1)
DASHBOARD_LEGACY_ROAD_RUNTIME_SOURCES = $(RK_ROOT)/lib/road_feature_runtime_fpga_stub.c
else
DASHBOARD_LEGACY_ROAD_RUNTIME_SOURCES = $(ROAD_RUNTIME_SOURCES)
endif
DASHBOARD_C_SOURCES = $(DASHBOARD_LEGACY_ROAD_RUNTIME_SOURCES) $(ADAS_RESULT_SOURCES) $(SNAPSHOT_SOURCES) $(DASHBOARD_ROAD_SOURCES) $(DASHBOARD_FPGA_ROAD_SOURCES) $(OSD_PROBE_SOURCES) $(UI_LAYOUT_SOURCES) $(RK_ROOT)/lib/video_dma_ui_buffers.c $(RK_ROOT)/lib/video_dma_dark.c $(RK_ROOT)/lib/video_dma_dark_async.c $(RK_ROOT)/lib/video_dma_dashboard_input.c $(RK_ROOT)/lib/detection_overlay.c $(DASHBOARD_YOLO_SOURCES) $(DASHBOARD_REF_SOURCES)
DASHBOARD_OBJECTS = $(patsubst $(RK_ROOT)/%.c,$(OUTPUT_ROOT)/dashboard/%.o,$(DASHBOARD_C_SOURCES))
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/detection_light_tracker.c
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/adas_decision.c
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/adas_signal_test.c
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/adas_lane_test.c
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/adas_pedestrian_test.c
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/pedestrian_alert.c
DASHBOARD_C_SOURCES += $(RK_ROOT)/lib/video_dma_sync_gpu.c
QT_CFLAGS = $(shell $(PKG_CONFIG) --cflags Qt5Gui)
QT_LIBS = $(shell $(PKG_CONFIG) --libs Qt5Gui)
.PHONY: dashboard test-dashboard-control
.PHONY: p2-status
.PHONY: fpga-probe
fpga-probe: $(OUTPUT_ROOT)/video_dma_fpga_probe
$(OUTPUT_ROOT)/video_dma_fpga_probe: $(RK_ROOT)/apps/video_dma_fpga_probe.c $(RK_ROOT)/include/fpga_road_uapi.h $(RK_ROOT)/include/video_dma_rgb_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"
p2-status: $(OUTPUT_ROOT)/video_dma_p2_status
$(OUTPUT_ROOT)/video_dma_p2_status: $(RK_ROOT)/apps/video_dma_p2_status.c
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror "$<" -o "$@"
dashboard: $(OUTPUT_ROOT)/video_dma_dashboard
.PHONY: specialist-dashboard-test light-specialist-probe
.PHONY: resident-light-display-test frozen-light-display-test
.PHONY: main-light-cpu-replay
main-light-cpu-replay: $(OUTPUT_ROOT)/test_main_light_cpu_replay
resident-light-display-test: $(OUTPUT_ROOT)/test_resident_light_display
frozen-light-display-test: $(OUTPUT_ROOT)/test_frozen_light_display
$(OUTPUT_ROOT)/test_main_light_cpu_replay $(OUTPUT_ROOT)/test_resident_light_display $(OUTPUT_ROOT)/test_frozen_light_display: $(OUTPUT_ROOT)/test_%: $(RK_ROOT)/tests/test_%.cpp $(RK_ROOT)/apps/video_dma_dashboard.cpp $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h) $(RK_ROOT)/lib/video_dma_specialist_batch.h $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm
.PHONY: pedestrian-dashboard-test
.PHONY: resident-pedestrian-replay
resident-pedestrian-replay: $(OUTPUT_ROOT)/test_resident_pedestrian_replay
$(OUTPUT_ROOT)/test_resident_pedestrian_replay: $(RK_ROOT)/tests/test_resident_pedestrian_replay.cpp $(RK_ROOT)/apps/video_dma_dashboard.cpp $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h) $(OUTPUT_ROOT)/dashboard/tests/specialist_main_yolo_fixture.o $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/tests/specialist_main_yolo_fixture.o $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm

.PHONY: test-resident-pedestrian-dashboard
test-resident-pedestrian-dashboard: $(OUTPUT_ROOT)/test_resident_pedestrian_dashboard
	QT_QPA_PLATFORM=offscreen "$(OUTPUT_ROOT)/test_resident_pedestrian_dashboard"
$(OUTPUT_ROOT)/test_resident_pedestrian_dashboard: $(RK_ROOT)/tests/test_resident_pedestrian_dashboard.cpp $(RK_ROOT)/tests/pedestrian_alert_fixture.h $(RK_ROOT)/apps/video_dma_dashboard.cpp $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h) $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm

pedestrian-dashboard-test: $(OUTPUT_ROOT)/test_pedestrian_dashboard
$(OUTPUT_ROOT)/test_pedestrian_dashboard: $(RK_ROOT)/tests/test_pedestrian_dashboard.cpp $(RK_ROOT)/apps/video_dma_dashboard.cpp $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h) $(OUTPUT_ROOT)/dashboard/tests/specialist_main_yolo_fixture.o $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/tests/specialist_main_yolo_fixture.o $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm

$(OUTPUT_ROOT)/video_dma_dashboard $(OUTPUT_ROOT)/test_dashboard_view $(OUTPUT_ROOT)/test_specialist_dashboard: $(RK_ROOT)/lib/video_dma_specialist_batch.h $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h)

.PHONY: test-crosswalk-display
.PHONY: test-vehicle-display
test-vehicle-display: $(OUTPUT_ROOT)/test_vehicle_display
	"$(OUTPUT_ROOT)/test_vehicle_display"
$(OUTPUT_ROOT)/test_vehicle_display: $(RK_ROOT)/tests/test_vehicle_display.cpp $(RK_ROOT)/lib/video_dma_dashboard_vehicle_display.h $(RK_ROOT)/lib/video_dma_dashboard_one_euro.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"

test-crosswalk-display: $(OUTPUT_ROOT)/test_crosswalk_display
	"$(OUTPUT_ROOT)/test_crosswalk_display"
$(OUTPUT_ROOT)/test_crosswalk_display: $(RK_ROOT)/tests/test_crosswalk_display.cpp $(RK_ROOT)/lib/video_dma_dashboard_crosswalk_display.h $(RK_ROOT)/lib/video_dma_dashboard_one_euro.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"

specialist-dashboard-test: $(OUTPUT_ROOT)/test_specialist_dashboard
$(OUTPUT_ROOT)/test_specialist_dashboard: $(RK_ROOT)/tests/test_specialist_dashboard.cpp $(RK_ROOT)/apps/video_dma_dashboard.cpp $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h) $(OUTPUT_ROOT)/dashboard/tests/specialist_main_yolo_fixture.o $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/tests/specialist_main_yolo_fixture.o $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm
$(OUTPUT_ROOT)/dashboard/lib/light_specialist.o: $(RK_ROOT)/lib/light_specialist.cpp $(RK_ROOT)/include/light_specialist.h $(RK_ROOT)/include/adas_signal_test.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -fPIC -pthread $(RKNN_CFLAGS) $(LIGHT_CV_CFLAGS) -I"$(RK_ROOT)/include" -c "$<" -o "$@"
light-specialist-probe: $(OUTPUT_ROOT)/light_specialist_probe
$(OUTPUT_ROOT)/light_specialist_probe: $(RK_ROOT)/apps/light_specialist_probe.cpp $(OUTPUT_ROOT)/dashboard/lib/light_specialist.o $(OUTPUT_ROOT)/dashboard/lib/light_specialist_color.o $(OUTPUT_ROOT)/dashboard/lib/traffic_light_color.o $(OUTPUT_ROOT)/dashboard/lib/adas_signal_test.o $(OUTPUT_ROOT)/dashboard/lib/adas_lane_test.o
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" "$<" $(filter %.o,$^) -o "$@" $(RKNN_LIBS) $(LIGHT_CV_LIBS) -lm
.PHONY: test-light-specialist-group
test-light-specialist-group: $(OUTPUT_ROOT)/test_light_specialist_group
	"$(OUTPUT_ROOT)/test_light_specialist_group"
$(OUTPUT_ROOT)/test_light_specialist_group: $(RK_ROOT)/tests/test_light_specialist_group.cpp $(RK_ROOT)/lib/light_specialist.cpp $(RK_ROOT)/include/light_specialist.h $(OUTPUT_ROOT)/dashboard/lib/light_specialist_color.o $(OUTPUT_ROOT)/dashboard/lib/traffic_light_color.o $(OUTPUT_ROOT)/dashboard/lib/adas_signal_test.o $(OUTPUT_ROOT)/dashboard/lib/adas_lane_test.o
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread $(RKNN_CFLAGS) $(LIGHT_CV_CFLAGS) -I"$(RK_ROOT)/include" "$<" $(filter %.o,$^) -o "$@" $(RKNN_LIBS) $(LIGHT_CV_LIBS) -lm
$(OUTPUT_ROOT)/dashboard/%.o: $(RK_ROOT)/%.c $(wildcard $(RK_ROOT)/include/*.h) $(wildcard $(RK_ROOT)/reference/road_ref/*.h)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -pthread $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(SNAPSHOT_CFLAGS) $(UI_LAYOUT_CFLAGS) -c "$<" -o "$@"
$(OUTPUT_ROOT)/dashboard/display.o: $(RK_ROOT)/apps/video_dma_display.c $(wildcard $(RK_ROOT)/include/*.h) $(wildcard $(RK_ROOT)/reference/road_ref/*.h)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DVDMA_DASHBOARD -Dmain=vdma_display_main $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(SNAPSHOT_CFLAGS) -c "$<" -o "$@"
$(OUTPUT_ROOT)/dashboard/reference/%.o: $(RK_ROOT)/reference/road_ref/%.cpp $(wildcard $(RK_ROOT)/reference/road_ref/*.h) $(RK_ROOT)/reference/road_ref/internal.hpp
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -fPIC -pthread $(ROAD_REF_CFLAGS) -c "$<" -o "$@"
$(OUTPUT_ROOT)/dashboard/reference/crosswalk.o: $(RK_ROOT)/reference/road_ref/third_party/pocketfft/pocketfft_hdronly.h
.PHONY: test-road-ref-adas
.PHONY: test-road-ref-lane-opening
.PHONY: test-road-ref-lane-roi
.PHONY: test-road-ref-normalization
test-road-ref-normalization: $(OUTPUT_ROOT)/test_road_ref_normalization
	"$(OUTPUT_ROOT)/test_road_ref_normalization"
$(OUTPUT_ROOT)/test_road_ref_normalization: $(RK_ROOT)/tests/test_road_ref_normalization.cpp $(RK_ROOT)/reference/road_ref/crosswalk.cpp $(RK_ROOT)/reference/road_ref/internal.hpp $(RK_ROOT)/reference/road_ref/road_ref.h $(RK_ROOT)/reference/road_ref/third_party/pocketfft/pocketfft_hdronly.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread $(ROAD_REF_CFLAGS) "$<" -o "$@" $(ROAD_REF_LIBS) -lm
test-road-ref-lane-roi: $(OUTPUT_ROOT)/test_road_ref_lane_roi
	"$(OUTPUT_ROOT)/test_road_ref_lane_roi"
$(OUTPUT_ROOT)/test_road_ref_lane_roi: $(RK_ROOT)/tests/test_road_ref_lane_roi.cpp $(RK_ROOT)/reference/road_ref/lane.cpp $(RK_ROOT)/reference/road_ref/internal.hpp $(RK_ROOT)/reference/road_ref/road_ref.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread $(ROAD_REF_CFLAGS) "$<" -o "$@" $(ROAD_REF_LIBS) -lm
test-road-ref-lane-opening: $(OUTPUT_ROOT)/test_road_ref_lane_opening
	"$(OUTPUT_ROOT)/test_road_ref_lane_opening"
$(OUTPUT_ROOT)/test_road_ref_lane_opening: $(RK_ROOT)/tests/test_road_ref_lane_opening.cpp $(RK_ROOT)/reference/road_ref/lane.cpp $(RK_ROOT)/reference/road_ref/internal.hpp $(RK_ROOT)/reference/road_ref/road_ref.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++17 -O2 -ffp-contract=off -Wall -Wextra -Werror -pthread $(ROAD_REF_CFLAGS) "$<" -o "$@" $(ROAD_REF_LIBS) -lm
.PHONY: test-road-ref-transport
.PHONY: test-road-ref-crc32
test-road-ref-crc32: $(OUTPUT_ROOT)/test_road_ref_crc32
	"$(OUTPUT_ROOT)/test_road_ref_crc32"
$(OUTPUT_ROOT)/test_road_ref_crc32: $(RK_ROOT)/tests/test_road_ref_crc32.c $(RK_ROOT)/include/road_ref_crc32.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -lz -o "$@"
test-road-ref-transport: $(OUTPUT_ROOT)/test_road_ref_transport
	"$(OUTPUT_ROOT)/test_road_ref_transport"
$(OUTPUT_ROOT)/test_road_ref_transport: $(RK_ROOT)/tests/test_road_ref_transport.c $(RK_ROOT)/lib/road_ref_transport.c $(wildcard $(RK_ROOT)/include/*.h)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -Wl,--wrap=ioctl -Wl,--wrap=mmap -Wl,--wrap=munmap -lz -o "$@"
test-road-ref-adas: $(OUTPUT_ROOT)/test_road_ref_adas
	"$(OUTPUT_ROOT)/test_road_ref_adas"
$(OUTPUT_ROOT)/dashboard/tests/test_road_ref_adas.o: $(RK_ROOT)/tests/test_road_ref_adas.c $(wildcard $(RK_ROOT)/include/*.h) $(wildcard $(RK_ROOT)/reference/road_ref/*.h)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -pthread -I"$(RK_ROOT)/include" -c "$<" -o "$@"
$(OUTPUT_ROOT)/test_road_ref_adas: $(OUTPUT_ROOT)/dashboard/tests/test_road_ref_adas.o $(OUTPUT_ROOT)/dashboard/lib/road_ref_adas.o $(OUTPUT_ROOT)/dashboard/lib/adas_result.o $$(DASHBOARD_REF_OBJECTS)
	$(CROSS_COMPILE)g++ -pthread $(filter %.o,$^) -o "$@" $(DASHBOARD_REF_LIBS) -lm
$(OUTPUT_ROOT)/video_dma_dashboard: $(RK_ROOT)/apps/video_dma_dashboard.cpp $(RK_ROOT)/lib/video_dma_dashboard_layers.h $(RK_ROOT)/lib/video_dma_dashboard_paint.h $(RK_ROOT)/lib/video_dma_dashboard_preview.h $(RK_ROOT)/lib/video_dma_dashboard_record.h $(RK_ROOT)/lib/video_dma_dashboard_wake.h $(RK_ROOT)/lib/video_dma_dashboard_sync.h $(RK_ROOT)/lib/video_dma_dashboard_history.h $(RK_ROOT)/lib/video_dma_dashboard_osd.h $(RK_ROOT)/include/video_dma_presentation.h $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm
test-dashboard-control: $(OUTPUT_ROOT)/test_dashboard_control
	"$(OUTPUT_ROOT)/test_dashboard_control"
$(OUTPUT_ROOT)/test_dashboard_control: $(RK_ROOT)/tests/test_dashboard_control.c $(RK_ROOT)/lib/video_dma_dark_async.c $(RK_ROOT)/lib/video_dma_dashboard_input.c $(wildcard $(RK_ROOT)/include/video_dma_dark*.h) $(RK_ROOT)/include/video_dma_dashboard_input.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

.PHONY: test-adas-signal-test
test-adas-signal-test: $(OUTPUT_ROOT)/test_adas_signal_test
	"$(OUTPUT_ROOT)/test_adas_signal_test"
$(OUTPUT_ROOT)/test_adas_signal_test: $(RK_ROOT)/tests/test_adas_signal_test.c $(RK_ROOT)/lib/adas_signal_test.c $(RK_ROOT)/lib/adas_lane_test.c $(RK_ROOT)/include/adas_signal_test.h $(RK_ROOT)/include/adas_lane_test.h $(RK_ROOT)/include/adas_decision.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: test-adas-lane-test
.PHONY: test-adas-pedestrian-test
.PHONY: test-pedestrian-alert
test-pedestrian-alert: $(OUTPUT_ROOT)/test_pedestrian_alert
	"$(OUTPUT_ROOT)/test_pedestrian_alert"
$(OUTPUT_ROOT)/test_pedestrian_alert: $(RK_ROOT)/tests/test_pedestrian_alert.c $(RK_ROOT)/tests/pedestrian_alert_fixture.h $(RK_ROOT)/lib/pedestrian_alert.c $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/include/pedestrian_alert.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

test-adas-pedestrian-test: $(OUTPUT_ROOT)/test_adas_pedestrian_test
	"$(OUTPUT_ROOT)/test_adas_pedestrian_test"
$(OUTPUT_ROOT)/test_adas_pedestrian_test: $(RK_ROOT)/tests/test_adas_pedestrian_test.c $(RK_ROOT)/lib/adas_pedestrian_test.c $(RK_ROOT)/include/adas_pedestrian_test.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm

test-adas-lane-test: $(OUTPUT_ROOT)/test_adas_lane_test
	"$(OUTPUT_ROOT)/test_adas_lane_test"
$(OUTPUT_ROOT)/test_adas_lane_test: $(RK_ROOT)/tests/test_adas_lane_test.c $(RK_ROOT)/lib/adas_lane_test.c $(RK_ROOT)/lib/adas_signal_test.c $(RK_ROOT)/include/adas_lane_test.h $(RK_ROOT)/include/adas_signal_test.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: test-adas-decision
test-adas-decision: $(OUTPUT_ROOT)/test_adas_decision
	"$(OUTPUT_ROOT)/test_adas_decision"
$(OUTPUT_ROOT)/test_adas_decision: $(RK_ROOT)/tests/test_adas_decision.c $(RK_ROOT)/lib/adas_decision.c $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/include/adas_decision.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

all: main

ADAS_RESULT_SOURCES := $(RK_ROOT)/lib/adas_result.c
ADAS_RESULT_HEADERS := $(RK_ROOT)/include/adas_result.h

OSD_PROBE_SOURCES := $(RK_ROOT)/lib/video_dma_osd_probe.c
OSD_PROBE_HEADERS := $(RK_ROOT)/include/video_dma_osd_probe.h
$(OUTPUT_ROOT)/video_dma_display $(OUTPUT_ROOT)/test_snapshot_display: $(OSD_PROBE_SOURCES) $(OSD_PROBE_HEADERS)

UI_LAYOUT_SOURCES := $(RK_ROOT)/lib/video_dma_ui_layout.c $(RK_ROOT)/vendor/cjson/cJSON.c
UI_LAYOUT_HEADERS := $(RK_ROOT)/include/video_dma_ui_layout.h $(RK_ROOT)/vendor/cjson/cJSON.h
UI_LAYOUT_CFLAGS := -I"$(RK_ROOT)/vendor/cjson" -DCJSON_NESTING_LIMIT=8
$(OUTPUT_ROOT)/video_dma_display $(OUTPUT_ROOT)/test_snapshot_display $(OUTPUT_ROOT)/test_osd_probe $(OUTPUT_ROOT)/test_ui_rect: $(UI_LAYOUT_SOURCES) $(UI_LAYOUT_HEADERS)
$(OUTPUT_ROOT)/video_dma_display $(OUTPUT_ROOT)/test_snapshot_display $(OUTPUT_ROOT)/test_osd_probe $(OUTPUT_ROOT)/test_ui_rect: DRM_CFLAGS += $(UI_LAYOUT_CFLAGS)

.PHONY: qt-ui-layout-image test-ui-layout test-ui-rect
qt-ui-layout-image: $(OUTPUT_ROOT)/qt_ui_layout_image
$(OUTPUT_ROOT)/ui_layout.o: $(RK_ROOT)/lib/video_dma_ui_layout.c $(UI_LAYOUT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC -I"$(RK_ROOT)/include" $(UI_LAYOUT_CFLAGS) -c "$<" -o "$@"
$(OUTPUT_ROOT)/ui_layout_cjson.o: $(RK_ROOT)/vendor/cjson/cJSON.c $(UI_LAYOUT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -fPIC $(UI_LAYOUT_CFLAGS) -c "$<" -o "$@"
$(OUTPUT_ROOT)/qt_ui_layout_image: $(RK_ROOT)/tools/qt_ui_layout_image.cpp $(OUTPUT_ROOT)/ui_layout.o $(OUTPUT_ROOT)/ui_layout_cjson.o $(UI_LAYOUT_HEADERS)
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -I"$(RK_ROOT)/include" $(shell $(PKG_CONFIG) --cflags Qt5Gui) $(filter %.cpp %.o,$^) -o "$@" $(shell $(PKG_CONFIG) --libs Qt5Gui) -lm
test-ui-layout: $(OUTPUT_ROOT)/test_ui_layout
	"$(OUTPUT_ROOT)/test_ui_layout"
$(OUTPUT_ROOT)/test_ui_layout: $(RK_ROOT)/tests/test_ui_layout.c $(UI_LAYOUT_SOURCES) $(UI_LAYOUT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(UI_LAYOUT_CFLAGS) $(filter %.c,$^) -o "$@" -lm
test-ui-rect: $(OUTPUT_ROOT)/test_ui_rect
	"$(OUTPUT_ROOT)/test_ui_rect"
$(OUTPUT_ROOT)/test_ui_rect: $(RK_ROOT)/tests/test_ui_rect.c $(OSD_PROBE_SOURCES) $(OSD_PROBE_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(filter %.c,$^) -Wl,--wrap=drmModeAtomicAlloc -Wl,--wrap=drmModeAtomicAddProperty -Wl,--wrap=drmModeAtomicCommit -Wl,--wrap=drmModeAtomicFree -Wl,--wrap=drmModeSetPlane -Wl,--wrap=drmModeObjectSetProperty -Wl,--wrap=drmModeRmFB -o "$@" $(DRM_LIBS)

.PHONY: qt-osd-test-image
qt-osd-test-image: $(OUTPUT_ROOT)/qt_osd_test_image
$(OUTPUT_ROOT)/qt_osd_test_image: $(RK_ROOT)/tools/qt_osd_test_image.cpp
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC $(shell $(PKG_CONFIG) --cflags Qt5Gui) "$<" -o "$@" $(shell $(PKG_CONFIG) --libs Qt5Gui)

.PHONY: test-osd-probe
test-osd-probe: $(OUTPUT_ROOT)/test_osd_probe
	"$(OUTPUT_ROOT)/test_osd_probe"
$(OUTPUT_ROOT)/test_osd_probe: $(RK_ROOT)/tests/test_osd_probe.c $(OSD_PROBE_SOURCES) $(OSD_PROBE_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(filter %.c,$^) -Wl,--wrap=drmModeSetPlane -Wl,--wrap=drmModeObjectSetProperty -Wl,--wrap=drmModeRmFB -Wl,--wrap=drmIoctl -Wl,--wrap=drmModeAtomicAlloc -Wl,--wrap=drmModeAtomicAddProperty -Wl,--wrap=drmModeAtomicCommit -Wl,--wrap=drmModeAtomicFree -o "$@" $(DRM_LIBS)

ROAD_RUNTIME_SOURCES := $(RK_ROOT)/lib/road_feature_runtime.c $(RK_ROOT)/lib/road_feature_consumer.c $(RK_ROOT)/lib/video_dma_features.c
ROAD_RUNTIME_HEADERS := $(RK_ROOT)/include/road_feature_runtime.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/video_dma_road_only.h $(RK_ROOT)/include/road_feature_transport.h $(RK_ROOT)/include/video_dma_feature_uapi.h $(RK_ROOT)/include/video_dma_features.h $(RK_ROOT)/include/video_dma_feature_validate.h
.PHONY: test-road-cv
test-road-cv: $(OUTPUT_ROOT)/test_road_cv
	"$(OUTPUT_ROOT)/test_road_cv"
$(OUTPUT_ROOT)/test_road_cv: $(RK_ROOT)/tests/test_road_cv.c $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/adas_result.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/video_dma_features.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: road-cv-replay
road-cv-replay: $(OUTPUT_ROOT)/road_cv_replay
$(OUTPUT_ROOT)/road_cv_replay: $(RK_ROOT)/apps/road_cv_replay.c $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/lib/road_feature_consumer.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/video_dma_features.h $(RK_ROOT)/include/video_dma_feature_validate.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
# Canvas replay: frozen 1920x1080 RGB565 frames through the production pair of
# calls (road_zebra_dense_extract -> road_cv_analyze) that road_zebra_adas.c runs
# live. Separate from road-cv-replay, which consumes decoded ROAD wire packets.
.PHONY: road-canvas-replay
road-canvas-replay: $(OUTPUT_ROOT)/road_canvas_replay
$(OUTPUT_ROOT)/road_canvas_replay: $(RK_ROOT)/apps/road_canvas_replay.c $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/lib/road_zebra_dense.c $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/road_zebra_dense.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/adas_result.h $(RK_ROOT)/include/road_wire.h $(RK_ROOT)/include/video_dma_features.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
# Board-side lane detector probe: packed 1920x1080 RGB565 canvases through
# road_lane_cv.c (the C port of scripts/road_roi_lane.py). Touches neither
# road_cv.c nor road_zebra_dense.c.
.PHONY: road-lane-probe
road-lane-probe: $(OUTPUT_ROOT)/road_lane_probe
$(OUTPUT_ROOT)/road_lane_probe: $(RK_ROOT)/apps/road_lane_probe.c $(RK_ROOT)/lib/road_lane_cv.c $(RK_ROOT)/include/road_lane_cv.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
.PHONY: road-osd-sim
road-osd-sim: $(OUTPUT_ROOT)/road_osd_sim
$(OUTPUT_ROOT)/road_osd_sim: $(RK_ROOT)/tools/road_osd_sim.c $(RK_ROOT)/lib/road_zebra_osd.c $(RK_ROOT)/include/road_zebra_osd.h $(RK_ROOT)/include/road_overlay_geometry.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: test-road-lane-cv
test-road-lane-cv: $(OUTPUT_ROOT)/test_road_lane_cv
	"$(OUTPUT_ROOT)/test_road_lane_cv"
$(OUTPUT_ROOT)/test_road_lane_cv: $(RK_ROOT)/tests/test_road_lane_cv.c $(RK_ROOT)/lib/road_lane_cv.c $(RK_ROOT)/include/road_lane_cv.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
LANE_CV_SOURCES := $(RK_ROOT)/lib/road_lane_cv.c
LANE_CV_HEADERS := $(RK_ROOT)/include/road_lane_cv.h
.PHONY: test-road-lane-output
test-road-lane-output: $(OUTPUT_ROOT)/test_road_lane_output
	"$(OUTPUT_ROOT)/test_road_lane_output"
$(OUTPUT_ROOT)/test_road_lane_output: $(RK_ROOT)/tests/test_road_lane_output.c $(RK_ROOT)/lib/road_zebra_osd.c $(RK_ROOT)/include/road_zebra_osd.h $(RK_ROOT)/include/road_overlay_geometry.h $(ADAS_RESULT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
ZEBRA_DENSE_SOURCES := $(RK_ROOT)/lib/road_zebra_dense.c
ZEBRA_DENSE_HEADERS := $(RK_ROOT)/include/road_zebra_dense.h
ZEBRA_ADAS_SOURCES := $(RK_ROOT)/lib/road_zebra_adas.c
ZEBRA_ADAS_HEADERS := $(RK_ROOT)/include/road_zebra_adas.h
# road_zebra_adas.c publishes zebra fields and calls road_lane_cv.c only for
# left/right geometry. Keep that dependency in the common source list so tests
# and the display target link the exact same implementation.
ZEBRA_SOURCES := $(ZEBRA_DENSE_SOURCES) $(ZEBRA_ADAS_SOURCES) $(LANE_CV_SOURCES) $(RK_ROOT)/lib/road_zebra_osd.c $(RK_ROOT)/lib/road_zebra_cv.c
ZEBRA_HEADERS := $(ZEBRA_DENSE_HEADERS) $(ZEBRA_ADAS_HEADERS) $(LANE_CV_HEADERS) $(RK_ROOT)/include/road_zebra_osd.h $(RK_ROOT)/include/road_overlay_geometry.h $(RK_ROOT)/include/road_zebra_cv.h

.PHONY: test-road-zebra-cv
test-road-zebra-cv: $(OUTPUT_ROOT)/test_road_zebra_cv
	"$(OUTPUT_ROOT)/test_road_zebra_cv"

$(OUTPUT_ROOT)/test_road_zebra_cv: $(RK_ROOT)/tests/test_road_zebra_cv.c $(RK_ROOT)/lib/road_zebra_cv.c $(RK_ROOT)/include/road_zebra_cv.h $(RK_ROOT)/include/adas_result.h
	@mkdir -p "$(OUTPUT_ROOT)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm

.PHONY: road-zebra-plane-replay
road-zebra-plane-replay: $(OUTPUT_ROOT)/road_zebra_plane_replay

$(OUTPUT_ROOT)/road_zebra_plane_replay: $(RK_ROOT)/apps/road_zebra_plane_replay.c $(RK_ROOT)/lib/road_zebra_cv.c $(RK_ROOT)/include/road_zebra_cv.h $(RK_ROOT)/include/adas_result.h
	@mkdir -p "$(OUTPUT_ROOT)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@" -lm
.PHONY: road-plane-adas-probe
road-plane-adas-probe: $(OUTPUT_ROOT)/road_plane_adas_probe
$(OUTPUT_ROOT)/road_plane_adas_probe: $(RK_ROOT)/apps/road_plane_adas_probe.c $$(SNAPSHOT_SOURCES) $$(SNAPSHOT_HEADERS) $(ZEBRA_SOURCES) $(ZEBRA_HEADERS) $$(ROAD_RUNTIME_SOURCES) $$(ADAS_RESULT_SOURCES) $$(ROAD_CV_SOURCES)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -o "$@" $(RGA_LIBS) -lm
# The zebra producer publishes through the ROAD ADAS bridge, so any binary that
# links it also needs the road_cv detector and its ADAS adapter.
ROAD_CV_SOURCES := $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/lib/road_cv_adas.c
ROAD_CV_HEADERS := $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/road_cv_adas.h
# The MinGW host toolchain lacks <linux/*.h>; the shim exists only for tests.
ZEBRA_TEST_CFLAGS := -pthread
ifneq (,$(findstring NT,$(shell uname -s 2>/dev/null)))
ZEBRA_TEST_CFLAGS += -I"$(RK_ROOT)/tests/host_shim"
# Native gcc ignores POSIX temp paths and falls back to C:\WINDOWS, which is not
# writable here, so give it a build-local temp directory when none is exported.
ifeq (,$(strip $(TEMP)$(TMP)))
HOST_TEMP := $(OUTPUT_ROOT)/tmp
$(shell mkdir -p "$(HOST_TEMP)")
export TEMP := $(HOST_TEMP)
export TMP := $(HOST_TEMP)
endif
endif
.PHONY: test-road-zebra-dense test-road-zebra-adas
test-road-zebra-dense: $(OUTPUT_ROOT)/test_road_zebra_dense
	"$(OUTPUT_ROOT)/test_road_zebra_dense"
$(OUTPUT_ROOT)/test_road_zebra_dense: $(RK_ROOT)/tests/test_road_zebra_dense.c $(ZEBRA_DENSE_SOURCES) $(ZEBRA_DENSE_HEADERS) $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
test-road-zebra-adas: $(OUTPUT_ROOT)/test_road_zebra_adas
	"$(OUTPUT_ROOT)/test_road_zebra_adas"
$(OUTPUT_ROOT)/test_road_zebra_adas: $(RK_ROOT)/tests/test_road_zebra_adas.c $(ZEBRA_SOURCES) $(ZEBRA_HEADERS) $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/lib/road_cv_adas.c $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/road_cv_adas.h $(RK_ROOT)/include/adas_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(ZEBRA_TEST_CFLAGS) $(filter %.c,$^) -o "$@" -lm
PYTHON ?= python
.PHONY: test-road-wire road-wire-check
test-road-wire: $(OUTPUT_ROOT)/test_road_wire
	"$(OUTPUT_ROOT)/test_road_wire"
$(OUTPUT_ROOT)/test_road_wire: $(RK_ROOT)/tests/test_road_wire.c $(RK_ROOT)/include/road_wire.h $(RK_ROOT)/include/road_cv.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
# The former dense-run/packet-bank FPGA test targets were removed with the
# retired road transport RTL. Their fixtures remain in the archived workspace;
# production FPGA_ROAD builds use the native HDMI result contract below.
$(OUTPUT_ROOT)/test_road_wire_compat: $(RK_ROOT)/tests/test_road_wire_compat.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/include/road_wire.h $(RK_ROOT)/include/video_dma_features.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: test-road-cv-adas
test-road-cv-adas: $(OUTPUT_ROOT)/test_road_cv_adas
	"$(OUTPUT_ROOT)/test_road_cv_adas"
$(OUTPUT_ROOT)/test_road_cv_adas: $(RK_ROOT)/tests/test_road_cv_adas.c $(RK_ROOT)/lib/road_cv_adas.c $(RK_ROOT)/lib/road_cv.c $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/include/road_cv_adas.h $(RK_ROOT)/include/road_cv.h $(RK_ROOT)/include/adas_result.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/video_dma_features.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: test-road-runtime
test-road-runtime: $(OUTPUT_ROOT)/test_road_runtime
	"$(OUTPUT_ROOT)/test_road_runtime"
$(OUTPUT_ROOT)/test_road_runtime: $(RK_ROOT)/tests/test_road_runtime.c $(ROAD_RUNTIME_SOURCES) $(ROAD_RUNTIME_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -Wl,--wrap=ioctl -o "$@"

$(OUTPUT_ROOT)/include/video_dma_road_only.h: $(RK_ROOT)/include/video_dma_road_only.h
	mkdir -p "$(@D)"
	cp "$<" "$@"

.PHONY: test-plane-uapi
.PHONY: plane-probe
plane-probe: $(OUTPUT_ROOT)/plane_probe
$(OUTPUT_ROOT)/plane_probe: $(RK_ROOT)/tools/plane_probe.c $(RK_ROOT)/include/video_dma_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"
test-plane-uapi: $(OUTPUT_ROOT)/test_plane_uapi
	"$(OUTPUT_ROOT)/test_plane_uapi"
$(OUTPUT_ROOT)/test_plane_uapi: $(RK_ROOT)/tests/test_plane_uapi.c $(RK_ROOT)/include/video_dma_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" -I"$(RK_ROOT)/tests/host_shim" "$<" -o "$@"

.PHONY: test-road-only-contract
test-road-only-contract: $(OUTPUT_ROOT)/test_road_only_contract
	"$(OUTPUT_ROOT)/test_road_only_contract"
$(OUTPUT_ROOT)/test_road_only_contract: $(RK_ROOT)/tests/test_road_only_contract.c $(RK_ROOT)/include/video_dma_road_only.h $(RK_ROOT)/include/video_dma_feature_validate.h $(RK_ROOT)/include/video_dma_feature_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" -I"$(RK_ROOT)/tests/host_shim" "$<" -o "$@"

modules: $(MODULE_DIR)/Makefile $(MODULE_DIR)/video_dma.c $(OUTPUT_ROOT)/include/video_dma_uapi.h $(OUTPUT_ROOT)/include/video_dma_feature_uapi.h $(OUTPUT_ROOT)/include/video_dma_feature_regs.h $(OUTPUT_ROOT)/include/video_dma_feature_validate.h
modules: $(OUTPUT_ROOT)/include/video_dma_snapshot_uapi.h $(OUTPUT_ROOT)/include/video_dma_snapshot_validate.h $(OUTPUT_ROOT)/include/video_dma_road_only.h
modules: $(OUTPUT_ROOT)/include/video_dma_dark_uapi.h $(OUTPUT_ROOT)/include/video_dma_dark_core.h
modules: $(OUTPUT_ROOT)/include/video_dma_road_ref_uapi.h $(OUTPUT_ROOT)/include/road_ref_wire.h $(MODULE_DIR)/video_dma_road_ref.inc
modules: $(OUTPUT_ROOT)/include/fpga_road_uapi.h $(MODULE_DIR)/video_dma_fpga_road.inc
modules: $(OUTPUT_ROOT)/include/video_dma_rgb_uapi.h $(MODULE_DIR)/video_dma_rgb_impl.h
	$(MAKE) -C "$(KDIR)" M="$(MODULE_DIR)" modules

$(OUTPUT_ROOT)/include/video_dma_dark_%.h: $(RK_ROOT)/include/video_dma_dark_%.h
	mkdir -p "$(@D)"
	cp "$<" "$@"

.PHONY: dark-control test-dark-control
dark-control: $(OUTPUT_ROOT)/video_dma_dark_control
$(OUTPUT_ROOT)/video_dma_dark_control: $(RK_ROOT)/apps/video_dma_dark_control.c $(RK_ROOT)/lib/video_dma_dark.c $(RK_ROOT)/include/video_dma_dark.h $(RK_ROOT)/include/video_dma_dark_uapi.h $(RK_ROOT)/include/video_dma_dark_core.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

test-dark-control: $(OUTPUT_ROOT)/test_dark_control
	"$(OUTPUT_ROOT)/test_dark_control"
$(OUTPUT_ROOT)/test_dark_control: $(RK_ROOT)/tests/test_dark_control.c $(RK_ROOT)/lib/video_dma_dark.c $(RK_ROOT)/include/video_dma_dark.h $(RK_ROOT)/include/video_dma_dark_core.h $(RK_ROOT)/include/video_dma_dark_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -Wl,--wrap=ioctl -o "$@"

$(OUTPUT_ROOT)/include/video_dma_snapshot_%.h: $(RK_ROOT)/include/video_dma_snapshot_%.h
	mkdir -p "$(@D)"
	cp "$<" "$@"

$(MODULE_DIR)/Makefile: $(RK_ROOT)/driver/Makefile
	mkdir -p "$(@D)"
	cp "$<" "$@"

$(MODULE_DIR)/video_dma.c: $(RK_ROOT)/driver/video_dma.c
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(MODULE_DIR)/video_dma_road_ref.inc: $(RK_ROOT)/driver/video_dma_road_ref.inc
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(OUTPUT_ROOT)/include/video_dma_road_ref_uapi.h: $(RK_ROOT)/include/video_dma_road_ref_uapi.h
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(OUTPUT_ROOT)/include/road_ref_wire.h: $(RK_ROOT)/include/road_ref_wire.h
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(MODULE_DIR)/video_dma_fpga_road.inc: $(RK_ROOT)/driver/video_dma_fpga_road.inc
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(MODULE_DIR)/video_dma_rgb_impl.h: $(RK_ROOT)/driver/video_dma_rgb_impl.h
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(OUTPUT_ROOT)/include/video_dma_rgb_uapi.h: $(RK_ROOT)/include/video_dma_rgb_uapi.h
	mkdir -p "$(@D)"
	cp "$<" "$@"
$(OUTPUT_ROOT)/include/fpga_road_uapi.h: $(RK_ROOT)/include/fpga_road_uapi.h
	mkdir -p "$(@D)"
	cp "$<" "$@"

$(OUTPUT_ROOT)/include/video_dma_uapi.h: $(RK_ROOT)/include/video_dma_uapi.h
	mkdir -p "$(@D)"
	cp "$<" "$@"

$(OUTPUT_ROOT)/include/video_dma_feature_%.h: $(RK_ROOT)/include/video_dma_feature_%.h
	mkdir -p "$(@D)"
	cp "$<" "$@"

display: $(OUTPUT_ROOT)/video_dma_display

display-sdk:
	test -n "$(SDK_ROOT)"
	$(MAKE) -C "$(RK_ROOT)" display DRM_CFLAGS="-I$(SDK_DRM_MODE_HEADERS) -I$(SDK_DRM_ROOT)/include/libdrm" DRM_LIBS="$(SDK_DRM_ROOT)/lib/arm64/libdrm.so"

$(OUTPUT_ROOT)/video_dma_display: $(RK_ROOT)/apps/video_dma_display.c $(ROAD_RUNTIME_SOURCES) $(ROAD_RUNTIME_HEADERS) $(ADAS_RESULT_SOURCES) $(ADAS_RESULT_HEADERS) $(RK_ROOT)/include/video_dma_frame.h $(RK_ROOT)/include/video_dma_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -o "$@" $(DRM_LIBS) $(RGA_LIBS) -lm

test-metadata: $(OUTPUT_ROOT)/test_frame_metadata

.PHONY: test-snapshot
test-snapshot: $(OUTPUT_ROOT)/test_snapshot_contract
$(OUTPUT_ROOT)/test_snapshot_contract: $(RK_ROOT)/tests/test_snapshot_contract.c $(RK_ROOT)/include/video_dma_snapshot_uapi.h $(RK_ROOT)/include/video_dma_snapshot_validate.h $(RK_ROOT)/include/video_dma_feature_uapi.h $(RK_ROOT)/include/video_dma_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"

SNAPSHOT_SOURCES := $(RK_ROOT)/lib/video_dma_snapshot.c $(RK_ROOT)/lib/video_dma_snapshot_rga.c $(RK_ROOT)/lib/video_dma_snapshot_async.c
.PHONY: rgb-pair-probe
rgb-pair-probe: $(OUTPUT_ROOT)/video_dma_rgb_pair_probe
$(OUTPUT_ROOT)/video_dma_rgb_pair_probe: $(RK_ROOT)/apps/video_dma_rgb_pair_probe.c $(SNAPSHOT_SOURCES) $$(SNAPSHOT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
.PHONY: test-snapshot-async
.PHONY: test-snapshot-rgb
test-snapshot-rgb: $(OUTPUT_ROOT)/test_snapshot_rgb
	"$(OUTPUT_ROOT)/test_snapshot_rgb"
$(OUTPUT_ROOT)/test_snapshot_rgb: $(RK_ROOT)/tests/test_snapshot_rgb.c $(RK_ROOT)/lib/video_dma_snapshot.c $(RK_ROOT)/lib/video_dma_snapshot_rgb_impl.h $(RK_ROOT)/lib/yolo_preprocess.c $(RK_ROOT)/lib/video_dma_snapshot_rga.c $(RK_ROOT)/include/video_dma_snapshot.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" "$<" $(RK_ROOT)/lib/yolo_preprocess.c $(RK_ROOT)/lib/video_dma_snapshot_rga.c -o "$@"
test-snapshot-async: $(OUTPUT_ROOT)/test_snapshot_async
	"$(OUTPUT_ROOT)/test_snapshot_async"
$(OUTPUT_ROOT)/test_snapshot_async: $(RK_ROOT)/tests/test_snapshot_async.c $(RK_ROOT)/lib/video_dma_snapshot_async.c $(RK_ROOT)/include/video_dma_snapshot_async.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
SNAPSHOT_HEADERS := $(RK_ROOT)/include/video_dma_snapshot.h $(RK_ROOT)/include/video_dma_snapshot_uapi.h $(RK_ROOT)/include/video_dma_snapshot_validate.h
SNAPSHOT_HEADERS += $(RK_ROOT)/include/video_dma_snapshot_rga.h
SNAPSHOT_HEADERS += $(RK_ROOT)/include/video_dma_rgb_uapi.h $(RK_ROOT)/lib/video_dma_snapshot_rgb_impl.h
SNAPSHOT_HEADERS += $(RK_ROOT)/include/video_dma_frame.h $(RK_ROOT)/include/video_dma_uapi.h $(RK_ROOT)/include/video_dma_feature_uapi.h
SNAPSHOT_HEADERS += $(RK_ROOT)/include/video_dma_road_ref_uapi.h $(RK_ROOT)/include/road_ref_wire.h $(RK_ROOT)/reference/road_ref/road_ref.h
$(OUTPUT_ROOT)/video_dma_display: $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS)
$(OUTPUT_ROOT)/video_dma_display: $(ZEBRA_SOURCES) $(ZEBRA_HEADERS) $(ROAD_CV_SOURCES) $(ROAD_CV_HEADERS)
test-snapshot: $(OUTPUT_ROOT)/test_snapshot_runtime $(OUTPUT_ROOT)/test_snapshot_plane
test-snapshot: $(OUTPUT_ROOT)/test_snapshot_reference
$(OUTPUT_ROOT)/test_snapshot_reference: $(RK_ROOT)/tests/test_snapshot_reference.c $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -o "$@" $(RGA_LIBS)
$(OUTPUT_ROOT)/test_snapshot_plane: $(RK_ROOT)/tests/test_snapshot_plane.c $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -Wl,--wrap=ioctl -Wl,--wrap=mmap -Wl,--wrap=munmap -o "$@" $(RGA_LIBS)
$(OUTPUT_ROOT)/test_snapshot_runtime: $(RK_ROOT)/tests/test_snapshot_runtime.c $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -Wl,--wrap=ioctl -o "$@" $(RGA_LIBS)

test-snapshot: $(OUTPUT_ROOT)/test_snapshot_display
test-snapshot: $(OUTPUT_ROOT)/test_snapshot_rga_runtime
.PHONY: snapshot-rga-board
snapshot-rga-board: $(OUTPUT_ROOT)/test_snapshot_rga_board
$(OUTPUT_ROOT)/test_snapshot_rga_board: $(RK_ROOT)/tests/test_snapshot_rga_board.c $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -o "$@" $(RGA_LIBS)
$(OUTPUT_ROOT)/test_snapshot_rga_runtime: $(RK_ROOT)/tests/test_snapshot_rga_runtime.c $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(SNAPSHOT_CFLAGS) $(filter %.c,$^) -Wl,--wrap=ioctl -Wl,--wrap=vdma_snapshot_rga_copy -Wl,--wrap=vdma_snapshot_rga_cpu_access -o "$@" $(RGA_LIBS)

test-perception: $(OUTPUT_ROOT)/test_perception_contract

.PHONY: test-adas-result
test-adas-result: $(OUTPUT_ROOT)/test_adas_result
	"$(OUTPUT_ROOT)/test_adas_result"

$(OUTPUT_ROOT)/test_adas_result: $(RK_ROOT)/tests/test_adas_result.c $(ADAS_RESULT_SOURCES) $(ADAS_RESULT_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

$(OUTPUT_ROOT)/test_snapshot_display: $(RK_ROOT)/tests/test_snapshot_display.c $(RK_ROOT)/apps/video_dma_display.c $(ROAD_RUNTIME_SOURCES) $(ROAD_RUNTIME_HEADERS) $(ADAS_RESULT_SOURCES) $(ADAS_RESULT_HEADERS) $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS) $(ZEBRA_SOURCES) $(ZEBRA_HEADERS) $(ROAD_CV_SOURCES) $(ROAD_CV_HEADERS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(SNAPSHOT_CFLAGS) $(filter-out $(RK_ROOT)/apps/video_dma_display.c,$(filter %.c,$^)) -Wl,--wrap=ioctl -Wl,--wrap=drmModeSetCrtc -o "$@" $(DRM_LIBS) $(RGA_LIBS) -lm
test-feature-validation: $(OUTPUT_ROOT)/test_feature_validation

test-road-features: $(OUTPUT_ROOT)/test_road_features

$(OUTPUT_ROOT)/test_road_features: $(RK_ROOT)/tests/test_road_features.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/lib/road_feature_consumer.c $(RK_ROOT)/lib/road_feature_transport.c $(RK_ROOT)/include/video_dma_features.h $(RK_ROOT)/include/video_dma_feature_validate.h $(RK_ROOT)/include/video_dma_feature_uapi.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/road_feature_transport.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

.PHONY: test-quad-color
test-quad-color: $(OUTPUT_ROOT)/test_quad_color_contract $(OUTPUT_ROOT)/test_feature_consumer $(OUTPUT_ROOT)/test_feature_events
.PHONY: test-single-features
test-single-features: $(OUTPUT_ROOT)/test_feature_consumer $(OUTPUT_ROOT)/test_feature_events $(OUTPUT_ROOT)/test_joint_consumer

$(OUTPUT_ROOT)/test_feature_events: $(RK_ROOT)/tests/test_feature_events.c $(RK_ROOT)/lib/video_dma_feature_consumer.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/include/video_dma_feature_consumer.h $(RK_ROOT)/include/video_dma_features.h $(RK_ROOT)/include/video_dma_feature_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -Wl,--wrap=ioctl -o "$@"

$(OUTPUT_ROOT)/test_feature_consumer: $(RK_ROOT)/tests/test_feature_consumer.c $(RK_ROOT)/lib/video_dma_feature_consumer.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/include/video_dma_feature_consumer.h $(RK_ROOT)/include/video_dma_features.h $(RK_ROOT)/include/video_dma_feature_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -Wl,--wrap=ioctl -o "$@"

$(OUTPUT_ROOT)/test_quad_color_contract: $(RK_ROOT)/tests/test_quad_color_contract.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/include/video_dma_features.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

$(OUTPUT_ROOT)/test_feature_validation: $(RK_ROOT)/tests/test_feature_validation.c $(RK_ROOT)/include/video_dma_feature_validate.h $(RK_ROOT)/include/video_dma_feature_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"

$(OUTPUT_ROOT)/test_perception_contract: $(RK_ROOT)/tests/test_perception_contract.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/lib/perception_result.c $(RK_ROOT)/include/video_dma_features.h $(RK_ROOT)/include/perception_result.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

control: $(OUTPUT_ROOT)/video_dma_control

$(OUTPUT_ROOT)/video_dma_control: $(RK_ROOT)/apps/video_dma_control.c $(RK_ROOT)/include/video_dma_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"

$(OUTPUT_ROOT)/test_frame_metadata: $(RK_ROOT)/tests/test_frame_metadata.c $(RK_ROOT)/include/video_dma_frame.h $(RK_ROOT)/include/video_dma_uapi.h $(RK_ROOT)/include/video_dma_feature_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"

package:
	test -f "$(MODULE_DIR)/video_dma.ko"
	test -x "$(OUTPUT_ROOT)/video_dma_display"
	mkdir -p "$(OUTPUT_ROOT)/deploy/tests"
	cp "$(MODULE_DIR)/video_dma.ko" "$(OUTPUT_ROOT)/video_dma_display" "$(OUTPUT_ROOT)/deploy/"
	if test -x "$(OUTPUT_ROOT)/video_dma_control"; then cp "$(OUTPUT_ROOT)/video_dma_control" "$(OUTPUT_ROOT)/deploy/"; fi
	# Remove retired binaries left by an earlier package into this generated directory.
	rm -f "$(OUTPUT_ROOT)/deploy/video_dma_features" "$(OUTPUT_ROOT)/deploy/color_regions_capture" "$(OUTPUT_ROOT)/deploy/joint_features_capture"
	if test -x "$(OUTPUT_ROOT)/road_features_capture"; then cp "$(OUTPUT_ROOT)/road_features_capture" "$(OUTPUT_ROOT)/deploy/"; fi
	cp "$(RK_ROOT)/scripts/run_display.sh" "$(OUTPUT_ROOT)/deploy/"
	cp "$(RK_ROOT)/tools/capture.py" "$(RK_ROOT)/tools/capture_mmap.py" "$(OUTPUT_ROOT)/deploy/"
	cp "$(RK_ROOT)/tools/frame_metadata.py" "$(OUTPUT_ROOT)/deploy/"
	cp "$(RK_ROOT)/tests/test_mmap.py" "$(RK_ROOT)/tests/test_display_queue.py" "$(OUTPUT_ROOT)/deploy/tests/"
	cp "$(RK_ROOT)/tests/test_frame_metadata.py" "$(OUTPUT_ROOT)/deploy/tests/"
	cp "$(RK_ROOT)/tests/test_snapshot_board.py" "$(OUTPUT_ROOT)/deploy/tests/"
	cp "$(RK_ROOT)/../docs/rk3568/DEPLOYMENT.md" "$(OUTPUT_ROOT)/deploy/README.md"
	chmod +x "$(OUTPUT_ROOT)/deploy/run_display.sh"

clean:
	rm -f "$(OUTPUT_ROOT)/qt_osd_test_image" "$(OUTPUT_ROOT)/test_osd_probe"
	rm -f "$(OUTPUT_ROOT)/test_road_runtime" "$(OUTPUT_ROOT)/test_road_only_contract"
	if test -f "$(MODULE_DIR)/Makefile"; then $(MAKE) -C "$(KDIR)" M="$(MODULE_DIR)" clean; fi
	rm -f "$(OUTPUT_ROOT)/video_dma_display" "$(OUTPUT_ROOT)/video_dma_control" "$(OUTPUT_ROOT)/test_frame_metadata" "$(OUTPUT_ROOT)/test_perception_contract"
	rm -f "$(OUTPUT_ROOT)/test_color_regions" "$(OUTPUT_ROOT)/color_regions_offline"
	rm -f "$(OUTPUT_ROOT)/test_feature_events" "$(OUTPUT_ROOT)/test_feature_consumer" "$(OUTPUT_ROOT)/test_quad_color_contract"
	rm -f "$(OUTPUT_ROOT)/test_color_runtime" "$(OUTPUT_ROOT)/test_color_config"
	rm -f "$(OUTPUT_ROOT)/color_regions_capture"
	rm -f "$(OUTPUT_ROOT)/test_color_display"
	rm -f "$(OUTPUT_ROOT)/test_snapshot_contract" "$(OUTPUT_ROOT)/test_snapshot_runtime" "$(OUTPUT_ROOT)/test_snapshot_display"
	rm -f "$(OUTPUT_ROOT)/test_snapshot_rga_runtime"
	rm -f "$(OUTPUT_ROOT)/test_snapshot_rga_board"
	rm -f "$(OUTPUT_ROOT)/test_adas_result"

.PHONY: road-features-capture
road-features-capture: $(OUTPUT_ROOT)/road_features_capture
$(OUTPUT_ROOT)/road_features_capture: $(RK_ROOT)/apps/road_features_capture.c $(RK_ROOT)/lib/road_feature_transport.c $(RK_ROOT)/lib/road_feature_consumer.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/include/road_feature_transport.h $(RK_ROOT)/include/video_dma_road_only.h $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/video_dma_feature_validate.h $(RK_ROOT)/include/video_dma_feature_uapi.h $(RK_ROOT)/include/video_dma_uapi.h $(RK_ROOT)/include/video_dma_frame.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"

# Historical wire parsers remain for archived packet/ABI compatibility tests only.
.PHONY: test-joint-consumer
test-joint-consumer: $(OUTPUT_ROOT)/test_joint_consumer
$(OUTPUT_ROOT)/test_joint_consumer: $(RK_ROOT)/tests/test_joint_consumer.c $(RK_ROOT)/lib/video_dma_feature_consumer.c $(RK_ROOT)/lib/video_dma_features.c $(RK_ROOT)/lib/road_feature_consumer.c $(RK_ROOT)/include/video_dma_feature_consumer.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" $(filter %.c,$^) -Wl,--wrap=ioctl -o "$@"
$(OUTPUT_ROOT)/test_feature_consumer $(OUTPUT_ROOT)/test_feature_events: $(RK_ROOT)/lib/road_feature_consumer.c $(RK_ROOT)/include/road_feature_consumer.h $(RK_ROOT)/include/video_dma_feature_validate.h

.PHONY: test-dashboard-display
test-dashboard-display: $(OUTPUT_ROOT)/test_dashboard_display $(OUTPUT_ROOT)/test_dashboard_display_fpga
	"$(OUTPUT_ROOT)/test_dashboard_display"
	"$(OUTPUT_ROOT)/test_dashboard_display_fpga"
$(OUTPUT_ROOT)/test_dashboard_display $(OUTPUT_ROOT)/test_dashboard_display_fpga: $(RK_ROOT)/tests/test_snapshot_display.c $(RK_ROOT)/apps/video_dma_display.c $(ROAD_RUNTIME_SOURCES) $(ROAD_RUNTIME_HEADERS) $(ADAS_RESULT_SOURCES) $(ADAS_RESULT_HEADERS) $(SNAPSHOT_SOURCES) $(SNAPSHOT_HEADERS) $(ZEBRA_SOURCES) $(ZEBRA_HEADERS) $(ROAD_CV_SOURCES) $(ROAD_CV_HEADERS) $(OSD_PROBE_SOURCES) $(OSD_PROBE_HEADERS) $(UI_LAYOUT_SOURCES) $(UI_LAYOUT_HEADERS) $(RK_ROOT)/include/video_dma_dashboard.h
$(OUTPUT_ROOT)/test_dashboard_display $(OUTPUT_ROOT)/test_dashboard_display_fpga: $(RK_ROOT)/lib/detection_overlay.c $(RK_ROOT)/include/detection_overlay.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -DVDMA_DASHBOARD $(if $(filter %_fpga,$@),-DVDMA_HAVE_FPGA_ROAD) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(SNAPSHOT_CFLAGS) $(UI_LAYOUT_CFLAGS) $(filter-out $(RK_ROOT)/apps/video_dma_display.c,$(filter %.c,$^)) -Wl,--wrap=ioctl -Wl,--wrap=drmModeSetCrtc -o "$@" $(DRM_LIBS) $(RGA_LIBS) -lm
$(OUTPUT_ROOT)/test_dashboard_display_fpga: $(RK_ROOT)/lib/fpga_road.c $(RK_ROOT)/lib/fpga_road_osd.c $(RK_ROOT)/include/fpga_road.h $(RK_ROOT)/include/fpga_road_uapi.h

.PHONY: test-dashboard-view
.PHONY: test-dashboard-sync test-fpga-road test-fpga-road-driver
test-fpga-road-driver: $(OUTPUT_ROOT)/test_fpga_road_driver
	"$(OUTPUT_ROOT)/test_fpga_road_driver"
$(OUTPUT_ROOT)/test_fpga_road_driver: $(RK_ROOT)/tests/test_fpga_road_driver.c $(RK_ROOT)/driver/video_dma_fpga_road.inc $(RK_ROOT)/include/fpga_road_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" "$<" -o "$@"
test-dashboard-sync: $(OUTPUT_ROOT)/test_dashboard_sync
	"$(OUTPUT_ROOT)/test_dashboard_sync"
$(OUTPUT_ROOT)/test_dashboard_sync: $(RK_ROOT)/tests/test_dashboard_sync.cpp $(RK_ROOT)/lib/video_dma_dashboard_sync.h $(RK_ROOT)/include/video_dma_sync_gpu.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" -I"$(RK_ROOT)/tests/host_shim" "$<" -o "$@"
test-fpga-road: $(OUTPUT_ROOT)/test_fpga_road
	"$(OUTPUT_ROOT)/test_fpga_road"
.PHONY: test-fpga-road-osd
test-fpga-road-osd: $(OUTPUT_ROOT)/test_fpga_road_osd
	"$(OUTPUT_ROOT)/test_fpga_road_osd"
$(OUTPUT_ROOT)/test_fpga_road_osd: $(RK_ROOT)/tests/test_fpga_road_osd.c $(RK_ROOT)/lib/fpga_road_osd.c $(RK_ROOT)/include/fpga_road_osd.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -I"$(RK_ROOT)/include" -I"$(RK_ROOT)/tests/host_shim" $(filter %.c,$^) -o "$@"
$(OUTPUT_ROOT)/test_fpga_road: $(RK_ROOT)/tests/test_fpga_road.c $(RK_ROOT)/lib/fpga_road.c $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/include/fpga_road.h $(RK_ROOT)/include/fpga_road_uapi.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" -I"$(RK_ROOT)/tests/host_shim" $(filter %.c,$^) -o "$@"
.PHONY: test-light-tracker
test-light-tracker: $(OUTPUT_ROOT)/test_light_tracker
	"$(OUTPUT_ROOT)/test_light_tracker"
$(OUTPUT_ROOT)/test_light_tracker: $(RK_ROOT)/tests/test_light_tracker.c $(RK_ROOT)/lib/detection_light_tracker.c $(RK_ROOT)/lib/detection_overlay.c $(RK_ROOT)/lib/adas_result.c $(RK_ROOT)/include/detection_light_tracker.h $(RK_ROOT)/include/detection_overlay.h
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)gcc -std=c11 -O2 -Wall -Wextra -Werror -pthread -I"$(RK_ROOT)/include" $(filter %.c,$^) -o "$@"
test-dashboard-view: $(OUTPUT_ROOT)/test_dashboard_view
	QT_QPA_PLATFORM=offscreen "$(OUTPUT_ROOT)/test_dashboard_view"
$(OUTPUT_ROOT)/test_dashboard_view: $(RK_ROOT)/tests/test_dashboard_view.cpp $(RK_ROOT)/apps/video_dma_dashboard.cpp $(wildcard $(RK_ROOT)/lib/video_dma_dashboard_*.h) $(RK_ROOT)/include/video_dma_presentation.h $(OUTPUT_ROOT)/dashboard/display.o $$(DASHBOARD_OBJECTS) $$(DASHBOARD_REF_OBJECTS)
	mkdir -p "$(@D)"
	$(CROSS_COMPILE)g++ -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -pthread -DQT_NO_KEYWORDS $(DASHBOARD_YOLO_CFLAGS) -I"$(RK_ROOT)/include" $(DRM_CFLAGS) $(QT_CFLAGS) "$<" $(OUTPUT_ROOT)/dashboard/display.o $(DASHBOARD_OBJECTS) $(DASHBOARD_REF_OBJECTS) -o "$@" $(QT_LIBS) $(DRM_LIBS) $(RGA_LIBS) $(DASHBOARD_YOLO_LIBS) $(DASHBOARD_REF_LIBS) -lm
