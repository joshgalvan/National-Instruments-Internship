//
// Copyright 2022 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: native_dma_if
//
// Description:
//
//   This is the interface for interactions between native_dma_master and all
//   DMA sources.
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

interface native_dma_if #(
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

  // Write
  logic          [AWIDTH-1:0] write_addr;
  logic          [CWIDTH-1:0] write_count;
  logic                       write_ctrl_valid;
  logic                       write_ctrl_ready;
  logic          [DWIDTH-1:0] write_data;
  logic                       write_data_valid;
  logic                       write_data_ready;

  // Read
  logic          [AWIDTH-1:0] read_addr;
  logic          [CWIDTH-1:0] read_count;
  logic                       read_ctrl_valid;
  logic                       read_ctrl_ready;
  logic          [DWIDTH-1:0] read_data;
  logic                       read_data_valid;
  logic                       read_data_ready;

  task automatic reset_all ();
    write_addr = {AWIDTH{1'b0}};
    write_count = 0;
    write_ctrl_valid = 0;
    write_data = {DWIDTH{1'b0}};
    write_data_valid = 0;
    read_addr = 0;
    read_count = 0;
    read_ctrl_valid = 0;
    read_data_ready = 0;
  endtask

  task automatic write_init (
    input [CWIDTH-1:0] count,
    input [AWIDTH-1:0] address
  );
  // Setup the write path for a given transaction. Called at the beginning of
  // each new write.
  //
    do @(posedge clk); while (!write_ctrl_ready);
    write_ctrl_valid = 1;
    write_count = count;
    write_addr = address;
    do @(posedge clk); while (write_ctrl_ready);
    write_ctrl_valid = 0;
    do @(posedge clk); while (!write_data_ready);
  endtask: write_init

  task automatic write (
    input      [CWIDTH-1:0] count,
    input [DRAM_DWIDTH-1:0] data
  );
  // This task will write a single DRAM_DWIDTH word to the interconnect. If
  // this task is called consecutively, the 'address' will only get updated if
  // it is the first call of a new write. The design updates the address on its
  // own for a whole transaction, but needs the starting address.
  //
    bit [CWIDTH-1:0] count_int;
    int counter;

    // This ensures that this task will only write a single DRAM_DWIDTH word,
    // but can also write a partial word if needed, like at the end of a
    // transaction.
    if (count > SOURCE_WRD_PER_DRAM_WRD) begin
      count_int = SOURCE_WRD_PER_DRAM_WRD;
      counter   = SOURCE_WRD_PER_DRAM_WRD - 1;
    end else begin
      // The 'counter' used here is to grab the LSB of the data that's left of
      // count. The LSB DWIDTH words get loaded onto 'data' first.
      count_int = count;
      counter   = count - 1;
    end

    // Give the design the first DWIDTH word from the data.
    write_data       = data[counter*DWIDTH+:DWIDTH];
    write_data_valid = 1;

    // Give the design every DWIDTH word following the first from the input
    // 'data'.
    repeat (count_int - 1) begin
      @(posedge clk);
      counter = counter - 1;
      write_data       = data[counter*DWIDTH+:DWIDTH];
      write_data_valid = 1;
    end
    @(posedge clk);
  endtask: write

  task automatic read_init (
    input [CWIDTH-1:0] count,
    input [AWIDTH-1:0] address
  );
  // Initialize handshaking for read process.
  //
    read_data_ready = 1'b1;
    do @(posedge clk); while (!read_ctrl_ready);
    read_ctrl_valid = 1;
    read_count = count;
    read_addr = address;
    do @(posedge clk); while (read_ctrl_ready);
    read_ctrl_valid = 0;
  endtask: read_init

endinterface: native_dma_if
