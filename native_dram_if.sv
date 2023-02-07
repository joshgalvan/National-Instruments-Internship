//
// Copyright 2022 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: native_dram_if
//
// Description:
//
//   This is the interface for all interactions between native_dma_master and
//   native_interconnect.
//
// Parameters:
//
//   AWIDTH       : Address width of the DMA source
//   DWIDTH       : Data width of the DMA source
//   DRAM_AWIDTH  : Address width of DRAM
//   DRAM_DWIDTH  : Data width of DRAM. DRAM_DWIDTH must always be greater than
//                  and evenly divisible by DWIDTH. For example: 512/64, etc.
//   CWIDTH       : Max count the DMA source can request in a single transaction.
//   BURST_LENGTH : Number of words sent to DRAM in a single clock cycle. The
//                  address for each consecutive dram_wdf_data must be
//                  incremented by BURST_LENGTH.
//

interface native_dram_if #(
  parameter AWIDTH       = 32,
  parameter DWIDTH       = 64,
  parameter DRAM_AWIDTH  = 32,
  parameter DRAM_DWIDTH  = 512,
  parameter CWIDTH       = 8,
  parameter BURST_LENGTH = 8
) (
  input clk,
  input rst
);

  localparam SOURCE_WRD_PER_DRAM_WRD = (DRAM_DWIDTH / DWIDTH);

  // Both
  logic                       init_calib_complete;
  logic     [DRAM_AWIDTH-1:0] dram_addr;
  logic                 [2:0] dram_cmd;
  logic                       dram_en;
  logic                       dram_rdy;

  // Write
  logic                       dram_wdf_end;
  logic                       dram_wdf_wren;
  logic     [DRAM_DWIDTH-1:0] dram_wdf_data;
  logic [(DRAM_DWIDTH/8)-1:0] dram_wdf_mask;
  logic                       dram_wdf_rdy;

  // Read
  logic     [DRAM_DWIDTH-1:0] dram_rd_data;
  logic                       dram_rd_data_valid;
  logic                       dram_rd_data_ready;

  task automatic reset_all ();
    dram_rdy = 0;
    dram_wdf_rdy = 0;
    dram_rd_data = 0;
    dram_rd_data_valid = 0;
    init_calib_complete = 0;
  endtask

  task automatic init ();
  // Initialize the design to be able to accept write or read processes.
  //
    @(posedge clk);
    init_calib_complete = 1;
    dram_rdy = 1;
    dram_wdf_rdy = 1;
  endtask: init

  task automatic send_read (
    input [DRAM_DWIDTH-1:0] data
  );
  // This task will send a single DRAM_DWIDTH word to native_dma_master.
  //
    dram_rd_data_valid = 1;
    dram_rd_data = data;
    @(posedge clk);
  endtask: send_read

  task automatic invalid_data (
    input int clock_cycles
  );
  // This task will hold dram_rd_data_valid low for a given number of clock
  // cycles. Only called after the design has just grabbed the current valid
  // word.
  //
    dram_rd_data_valid = 0;
    repeat (clock_cycles) @(posedge clk);
    dram_rd_data_valid = 1;
  endtask

endinterface: native_dram_if
