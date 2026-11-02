set project_root [file normalize [file dirname [info script]]]
add_design [file join $project_root "rtl/features/road_plane_cdc.v"] -library work
add_design [file join $project_root "rtl/features/road_plane_reader.v"] -library work
add_design [file join $project_root "rtl/road/crosswalk_white_detect.v"] -library work
add_design [file join $project_root "rtl/road/fpga_road_capture.v"] -library work
add_design [file join $project_root "rtl/road/fpga_road_channel.v"] -library work
add_design [file join $project_root "rtl/road/lane_contour_samples.v"] -library work
add_design [file join $project_root "rtl/road/lane_delay_taps.v"] -library work
add_design [file join $project_root "rtl/road/lane_gap_evidence.v"] -library work
add_design [file join $project_root "rtl/road/lane_line_detect.v"] -library work
add_design [file join $project_root "rtl/road/lane_row_threshold.v"] -library work
add_design [file join $project_root "rtl/road/lane_segment_relation.v"] -library work
add_design [file join $project_root "rtl/road/lane_slot_allocator.v"] -library work
add_design [file join $project_root "rtl/road/lane_temporal_overlap.v"] -library work
add_design [file join $project_root "rtl/road/lane_white_ridge.v"] -library work
add_design [file join $project_root "rtl/video/video_token_fifo_bram.v"] -library work
add_design [file join $project_root "rtl/features/rgb_small_capture.v"] -library work
add_design [file join $project_root "rtl/features/rgb_small_writer.v"] -library work
add_design [file join $project_root "rtl/features/rgb_small_channel.v"] -library work
add_design [file join $project_root "rtl/features/rgb_small_ctl.v"] -library work
add_design [file join $project_root "rtl/video_dma/tx_packet_mux4.v"] -library work
add_design [file join $project_root "rtl/features/road_plane_tx.v"] -library work
add_design [file join $project_root "rtl/features/feature_pio_mux.v"] -library work

add_design [file join $project_root "rtl/control/dark_enhance_ctrl.v"] -library work
add_design [file join $project_root "rtl/control/dark_enhance_frame_tracker.v"] -library work
add_design [file join $project_root "rtl/control/dark_enhance_snapshot_cdc.v"] -library work
add_design [file join $project_root "rtl/control/dark_enhance_control_bank.v"] -library work
add_design [file join $project_root "rtl/video/dark_enhance_rgb565_pipeline.v"] -library work
add_design [file join $project_root "rtl/video/dark_enhance_canvas_pipeline.v"] -library work
add_design [file join $project_root "rtl/features/feature_control_mux.v"] -library work

add_design [file join $project_root "rtl/control/stage1_status_cdc.v"] -library work
add_design [file join $project_root "rtl/control/hdmi_native_status.v"] -library work
add_design [file join $project_root "rtl/video/stage1_binarization.v"] -library work
add_design [file join $project_root "rtl/video/stage1_gauss_filter.v"] -library work
add_design [file join $project_root "rtl/video/stage1_matrix.v"] -library work
add_design [file join $project_root "rtl/video/stage1_rgb_to_ycbcr.v"] -library work
add_design [file join $project_root "rtl/video/stage1_sobel.v"] -library work
add_design [file join $project_root "rtl/ethernet/eth_udp_rx.v"] -library work
add_design [file join $project_root "rtl/ethernet/eth_video_decode.v"] -library work
add_design [file join $project_root "rtl/ethernet/eth_arp_tx.v"] -library work
add_design [file join $project_root "rtl/ethernet/eth_token_fifo.v"] -library work
add_design [file join $project_root "rtl/video/hdmi_native_capture.v"] -library work
add_design [file join $project_root "rtl/video/hdmi_native_writer.v"] -library work
add_design [file join $project_root "rtl/video/hdmi_native_reader.v"] -library work
add_design [file join $project_root "rtl/video/hdmi_native_source.v"] -library work
add_design [file join $project_root "rtl/ethernet/eth_ch3_writer.v"] -library work
add_design [file join $project_root "rtl/ethernet/eth_ch3_input.v"] -library work
add_design [file join $project_root "rtl/ethernet/rgmii_interface.v"] -library work
add_design [file join $project_root "rtl/video_dma/video_dma_endpoint.v"] -library work
add_design [file join $project_root "rtl/video_dma/video_dma_frame_store.v"] -library work
add_design [file join $project_root "rtl/video_dma/video_dma_burst_mux.v"] -library work
add_design [file join $project_root "rtl/video_dma/video_dma_tx.v"] -library work
add_design [file join $project_root "rtl/pcie/interface_conversion/rt_cpld_16bit_ram_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/interface_conversion/rt_cpld_128_wr_finish.v"] -library work
add_design [file join $project_root "rtl/pcie/interface_conversion/rt_gs_dma_interface_conversion.v"] -library work
add_design [file join $project_root "rtl/pcie/interface_conversion/rt_mwr_16bit_ram_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rt_dma.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rt_pcie_dma.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rx/ipm_distributed_sdpram_v1_2.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rx/rt01_dma_rx.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rx/rt0101_dma_request_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rx/rt0102_dma_cpld_tag_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/pgs_pciex4_prefetch_fifo_v1_2.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt02_dma_tx.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt0201_dma_mwr_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt0202_dma_mrd_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt0203_dma_rd_ram.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt020301_dma_rd_ram_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/fifo/ipm_distributed_sdpram_v1_2_distributed_fifo.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/fifo/pgs_pciex4_fifo_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/fifo/pgs_pciex4_fifo_v1_2.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/rx/rt0103_dma_data_wr_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt0204_pio_cpld_ctrl.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt0205_pio_rd_ram.v"] -library work
add_design [file join $project_root "rtl/pcie/dma_src/tx/rt0206_data_mux.v"] -library work
add_design [file join $project_root "rtl/pcie/bridge_src/rt_bridge_top_v1_3_vpAll.vp"] -library work
add_design [file join $project_root "rtl/top/video_dma_hdmi_top.v"] -library work
add_design [file join $project_root "rtl/video/cmos_8_16bit.v"] -library work
add_design [file join $project_root "rtl/video/cmos_tailor.v"] -library work
add_design [file join $project_root "rtl/control/i2c_com.v"] -library work
add_design [file join $project_root "rtl/control/iic_dri.v"] -library work
add_design [file join $project_root "rtl/control/ms72xx_ctl.v"] -library work
add_design [file join $project_root "rtl/control/ms7200_ctl.v"] -library work
add_design [file join $project_root "rtl/control/ms7210_ctl.v"] -library work
add_design [file join $project_root "rtl/control/power_on_delay.v"] -library work
add_design [file join $project_root "rtl/ddr/rd_buf.v"] -library work
add_design [file join $project_root "rtl/ddr/rd_ctrl.v"] -library work
add_design [file join $project_root "rtl/control/reg_config.v"] -library work
add_design [file join $project_root "rtl/video/sync_vg.v"] -library work
add_design [file join $project_root "rtl/video/single_channel_fullres_display.v"] -library work
add_design [file join $project_root "rtl/video/video_rect_read_data_m0.v"] -library work
add_design [file join $project_root "rtl/video/video_rect_read_data_m1.v"] -library work
add_design [file join $project_root "rtl/video/timing_gen_xy.v"] -library work
add_design [file join $project_root "rtl/video/quad_channel_display.v"] -library work
add_design [file join $project_root "rtl/control/key_state_ctrl.v"] -library work
add_design [file join $project_root "rtl/video/display_process.v"] -library work
add_design [file join $project_root "rtl/ddr/frame_read_write.v"] -library work
add_design [file join $project_root "rtl/ddr/frame_fifo_read.v"] -library work
add_design [file join $project_root "rtl/ddr/frame_fifo_write.v"] -library work
add_design [file join $project_root "rtl/ddr/mem_read_arbi.v"] -library work
add_design [file join $project_root "rtl/ddr/mem_write_arbi.v"] -library work
add_design [file join $project_root "rtl/ddr/aq_axi_master_256.v"] -library work
add_design [file join $project_root "rtl/ddr/cmos_write_req_gen.v"] -library work
add_design [file join $project_root "rtl/ddr/wr_buf.v"] -library work
add_design [file join $project_root "rtl/ddr/wr_cmd_trans.v"] -library work
add_design [file join $project_root "rtl/ddr/wr_ctrl.v"] -library work
add_design [file join $project_root "rtl/ddr/wr_rd_ctrl_top.v"] -library work
add_design [file join $project_root "rtl/control/key_debounce.v"] -library work
add_design [file join $project_root "ip/PCIe_IP/PCIe_IP.idf"]
add_design [file join $project_root "ip/ram_16in_16out/ram_16in_16out.idf"]
add_design [file join $project_root "ip/ram_16in_128out/ram_16in_128out.idf"]
add_design [file join $project_root "ip/ram_128in_16out/ram_128in_16out.idf"]
add_design [file join $project_root "ip/ram_128in_128out/ram_128in_128out.idf"]
add_design [file join $project_root "ip/pll/pll.idf"]
add_design [file join $project_root "ip/rd_fram_buf/rd_fram_buf.idf"]
add_design [file join $project_root "ip/wr_fram_buf/wr_fram_buf.idf"]
add_design [file join $project_root "ip/ddr3_test/ddr3_test.idf"]
add_design [file join $project_root "ip/afifo_16i_64o_512/afifo_16i_64o_512.idf"]
add_design [file join $project_root "ip/afifo_64i_16o_128/afifo_64i_16o_128.idf"]
add_constraint [file join $project_root "constraints/pcie_endpoint.fdc"]
add_constraint [file join $project_root "constraints/ethernet_ch3.fdc"]
add_constraint [file join $project_root "constraints/video_dma_hdmi.fdc"]
add_constraint [file join $project_root "constraints/dark_enhance.fdc"]

# Fixed-seed implementation for PG2L100H / FBG676 / -6.
# Keep multi-corner timing analysis, hold fixing and recovery optimization.
set_arch -family Logos2 -device PG2L100H -speedgrade -6 -package FBG676
compile -top_module {video_dma_hdmi_top}
synthesize -ads -selected_syn_tool_opt 2 -report_exception TRUE -max_path 500
dev_map

# Load the combined physical constraints after device mapping.
set pcf_file [file join $project_root "constraints/global_shift.pcf"]
puts "Loading combined PCF $pcf_file"
add_constraint $pcf_file

# Load the DDR PHY routing constraints.
set rcf_file [file join $project_root "constraints/ddr_prior_route.rcf"]
puts "Loading RCF $rcf_file"
add_constraint $rcf_file

# Seed 1 / four threads passed reported internal timing.
# External IO, DDR exceptions and CDC/RDC still require independent signoff.
# PDS 2023.2-SP3 supports up to four internal router threads.
set pnr_rc [catch {pnr -optimize_multi_corner_timing -fix_hold_violation \
    -gplace_seed 1 -gplace_times 1 -seed_step 1 -saved_outcome 1 -parallel 1 \
    -number_of_router_thread 4 -route_mode normal \
    -optimize_gp_recovery -optimize_dp_recovery -optimize_rp_recovery \
    -optimize_routing_recovery TRUE -share_router_control_signal FALSE} pnr_msg]
puts "PNR result=$pnr_rc msg=$pnr_msg"
if {$pnr_rc != 0} {
    error "PNR failed; stopping without changing implementation strategy: $pnr_msg"
}

report_timing -report_exception TRUE -max_path 500

# Generate the bitstream after timing analysis.
set gen_bit 1
if {$gen_bit} {
    gen_bit_stream
}

# End the implementation flow.
return
