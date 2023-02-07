//
// Copyright 2022 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: native_dma_master_tb
//
// Description:
//
//   This testbench is the top testbench for native_dma_master.sv
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

`timescale 1ns / 1ps

module native_dma_master_tb #(
  parameter DWIDTH       = 64,
  parameter AWIDTH       = 32,
  parameter DRAM_AWIDTH  = 32,
  parameter DRAM_DWIDTH  = 512,
  parameter CWIDTH       = 8,
  parameter BURST_LENGTH = 8
);

  `include "test_exec.svh"
  import PkgTestExec::*;
  import PkgRandom::*;

  localparam SOURCE_WRD_PER_DRAM_WRD = (DRAM_DWIDTH / DWIDTH);
  localparam real CLK_PER = 5.0; // 200 MHz

  bit clk;
  bit rst;

  sim_clock_gen #(.PERIOD(CLK_PER), .AUTOSTART(0))
    clk_gen (.clk(clk), .rst(rst));

  native_dma_if #(
    .AWIDTH (AWIDTH),
    .DWIDTH (DWIDTH),
    .DRAM_AWIDTH (DRAM_AWIDTH),
    .DRAM_DWIDTH (DRAM_DWIDTH),
    .CWIDTH (CWIDTH)
  ) dma (.clk(clk), .rst(rst));

  native_dram_if #(
    .AWIDTH (AWIDTH),
    .DWIDTH (DWIDTH),
    .DRAM_AWIDTH (DRAM_AWIDTH),
    .DRAM_DWIDTH (DRAM_DWIDTH),
    .CWIDTH (CWIDTH)
  ) dram (.clk(clk), .rst(rst));

  native_dma_master dut (
    .clk                 (clk),
    .rst                 (rst),
    .write_addr          (dma.write_addr),
    .write_count         (dma.write_count),
    .write_ctrl_valid    (dma.write_ctrl_valid),
    .write_ctrl_ready    (dma.write_ctrl_ready),
    .write_data          (dma.write_data),
    .write_data_valid    (dma.write_data_valid),
    .write_data_ready    (dma.write_data_ready),
    .read_addr           (dma.read_addr),
    .read_count          (dma.read_count),
    .read_ctrl_valid     (dma.read_ctrl_valid),
    .read_ctrl_ready     (dma.read_ctrl_ready),
    .read_data           (dma.read_data),
    .read_data_valid     (dma.read_data_valid),
    .read_data_ready     (dma.read_data_ready),
    .init_calib_complete (dram.init_calib_complete),
    .dram_addr           (dram.dram_addr),
    .dram_cmd            (dram.dram_cmd),
    .dram_en             (dram.dram_en),
    .dram_rdy            (dram.dram_rdy),
    .dram_wdf_end        (dram.dram_wdf_end),
    .dram_wdf_wren       (dram.dram_wdf_wren),
    .dram_wdf_data       (dram.dram_wdf_data),
    .dram_wdf_mask       (dram.dram_wdf_mask),
    .dram_wdf_rdy        (dram.dram_wdf_rdy),
    .dram_rd_data        (dram.dram_rd_data),
    .dram_rd_data_valid  (dram.dram_rd_data_valid),
    .dram_rd_data_ready  (dram.dram_rd_data_ready)
  );

  function void write_gen_data (
    input [CWIDTH-1:0] count,
    input              random,
    input      mailbox m_box_checker_dram,
    input      mailbox m_box_to_design
  );
  // Produces two identical mailboxes each of DRAM_DWIDTH data at each index.
  //
  //   count              : Number of DWIDTH words needed to be created for
  //                        this transaction.
  //   random             : Bit to select if the generation will be random (1)
  //                        or sequential (0).
  //   m_box_checker_dram : This is the mailbox that holds DRAM_DWIDTH data to
  //                        be used to check with the output data of the
  //                        design.
  //   m_box_to_design    : This is the mailbox that outputs data to the
  //                        design.
  //
    bit      [DWIDTH-1:0] adder;
    bit      [DWIDTH-1:0] source_word;
    bit [DRAM_DWIDTH-1:0] dram_word;
    int                   repeat_count;

    if (random) begin
      adder       = Rand#(DWIDTH)::rand_bit();
      source_word = Rand#(DWIDTH)::rand_bit();
    end else begin
      adder       = 1;
      source_word = {{(DWIDTH-1){1'b0}}, 1'b1};
    end

    dram_word = {DRAM_DWIDTH{1'b0}};
    repeat_count = 0;

    repeat (count) begin
      repeat_count = repeat_count + 1;

      dram_word = {dram_word[DRAM_DWIDTH-DWIDTH-1:0], source_word};
      if (repeat_count % SOURCE_WRD_PER_DRAM_WRD == 0
      || repeat_count == count) begin
        void'(m_box_checker_dram.try_put(dram_word));
        void'(m_box_to_design.try_put(dram_word));
      end

      source_word = source_word + adder;
    end
  endfunction: write_gen_data

  task automatic write (
    input [CWIDTH-1:0] count,
    input              random,
    input          int underflow
  );
  // Simulate writes.
  //
  //   count     : Number of DWIDTH words for the transaction.
  //   random    : Bit to select if it's a random write or sequential write.
  //   underflow : Bit to select if this call will test underflow an underflow
  //               write or not.
  //
    bit      [AWIDTH-1:0] address;
    bit      [CWIDTH-1:0] count_int;
    bit [DRAM_DWIDTH-1:0] m_box_checker_wrd;
    bit [DRAM_DWIDTH-1:0] wrd_to_design;
    int                   repeat_count;
    int                   count_int_cache;   // Allows us to know if we'll have
                                             // a partial word eventually and
                                             // how much we need to write for
                                             // the last DRAM_DWIDTH word.

    mailbox m_box_checker   = new();
    mailbox m_box_to_design = new();

    bit done = 0;
    int wren_count = 0;

    if (random) begin
      address   = Rand#(AWIDTH)::rand_bit();
      count_int = Rand#(CWIDTH)::rand_bit();
    end else begin
      address   = 1;
      count_int = count;
    end

    count_int_cache = count_int;
    repeat_count = $ceil(real'(count_int) / SOURCE_WRD_PER_DRAM_WRD);

    write_gen_data(count_int, random, m_box_checker, m_box_to_design);
    $display("\nCurrent count       = %-d", count_int);

    dma.write_init(count_int, address);
    fork
      begin
      // Make the design write data.
        repeat (repeat_count) begin
          m_box_to_design.get(wrd_to_design);
          dma.write(count_int_cache, wrd_to_design);
          count_int_cache = count_int_cache - SOURCE_WRD_PER_DRAM_WRD;
          // Deassert dram_rdy and dram_wdf_rdy AFTER every full DRAM_DWIDTH
          // given to the design. This is the only time the interconnect will
          // interrupt the design.
          if (underflow > 0) begin
            dram.dram_rdy = 0;
            repeat (underflow) @(posedge clk);
            dram.dram_rdy = 1;
          end
        end
        dma.write_data_valid = 0;
        if (underflow > 0) begin
          dram.dram_rdy = 0;
          repeat (underflow) @(posedge clk);
          dram.dram_rdy = 1;
        end
        repeat ((count_int % SOURCE_WRD_PER_DRAM_WRD) + 1) @(posedge clk);
        done = 1;
      end
      begin
      // Check if dram_wdf_wren gets asserted the correct number of times.
        do begin
          @(negedge clk);
          if (dram.dram_rdy && dram.dram_wdf_wren) begin
            wren_count = wren_count + 1;
          end
        end while (!done);
        // $display("ACTUAL   wren_count = %-d", wren_count);
        // $display("EXPECTED wren_count = %-d\n", $ceil(real'(count_int) / SOURCE_WRD_PER_DRAM_WRD));
        `ASSERT_ERROR(wren_count == ($ceil(real'(count_int) / SOURCE_WRD_PER_DRAM_WRD)),
        "dram_wdf_wren was not asserted enough times.");
      end
      begin
      // Data integrity check of design.
        do begin
          @(negedge clk);
          if (dram.dram_rdy && dram.dram_wdf_wren) begin
            void'(m_box_checker.try_get(m_box_checker_wrd));
            $display("ACTUAL   write data = %h", dram.dram_wdf_data);
            $display("EXPECTED write data = %h", m_box_checker_wrd);
            `ASSERT_ERROR(m_box_checker_wrd == dram.dram_wdf_data,
            "Output write data does not match input write data");
          end
        end while (!done);
      end
    join
  endtask

  task automatic write_underflow_multiple ();
  // Test regular underflow writes and underflow random writes. This tests
  // both underflow from the DRAM/interconnect and from the DMA source.
  //
    int dram_low_time;
    int count;

    for (dram_low_time = 1; dram_low_time <= 5; dram_low_time++) begin
      for (count = 1; count <= 32; count++) begin
        // Test counts 1 through 16 with dram_low_time 1 through 5.
        write(count, 0, dram_low_time);
      end
      // Test 5 random write transactions with dram_low_time 1 through 5.
      write(count, 1, dram_low_time);
    end
  endtask: write_underflow_multiple

  task automatic write_multiple ();
  // Test regular writes and random writes.
  //
    int count;

    for (count = 1; count <= 32; count++) begin
      // Regular writes at a given count.
      write(count, 0, 0);
    end
    // 10 random writes.
    repeat (10) write(count, 1, 0);
  endtask: write_multiple

function void read_gen_data (
    input [CWIDTH-1:0] count,
    input              random,
    input      mailbox m_box_checker_source,
    input      mailbox m_box_to_design
  );
  // Produces two identical mailboxes each of DRAM_DWIDTH data at each index.
  //
  //   count                   : Number of DWIDTH words that need to be
  //                             generated for the transaction.
  //   random                  : Bit to select if the generation will be random
  //                             or not.
  //   m_box_to_checker_source : Mailbox the holds DWIDTH (source) data to be
  //                             be checked with the output data of the design.
  //   m_box_to_design         : Mailbox that holds the data to be output to
  //                             the design.
  //
    bit      [DWIDTH-1:0] adder;
    bit      [DWIDTH-1:0] source_word;
    bit [DRAM_DWIDTH-1:0] dram_word;
    int                   repeat_count;

    if (random) begin
      adder       = Rand#(DWIDTH)::rand_bit();
      source_word = Rand#(DWIDTH)::rand_bit();
    end else begin
      adder       = 1;
      source_word = {{(DWIDTH-1){1'b0}}, 1'b1};
    end

    dram_word = {DRAM_DWIDTH{1'b0}};
    repeat_count = 0;

    repeat (count) begin
      repeat_count = repeat_count + 1;

      dram_word = {dram_word[DRAM_DWIDTH-DWIDTH-1:0], source_word};
      if (repeat_count % SOURCE_WRD_PER_DRAM_WRD == 0
      || repeat_count == count) begin
        void'(m_box_to_design.try_put(dram_word));
      end

      void'(m_box_checker_source.try_put(source_word));
      source_word = source_word + adder;
    end
  endfunction: read_gen_data

  task automatic read (
    input [CWIDTH-1:0] count,
    input              random,
    input          int underflow,
    input              source_underflow
  );
  // Simulate read process.
  //
  //   count     : Number of DWIDTH words for this transaction.
  //   random    : Bit to select if the transaction will be random or not.
  //   underflow : Bit to select if the transaction will test underflow or not.
  //
    bit      [AWIDTH-1:0] address;
    bit      [CWIDTH-1:0] count_int;
    bit      [DWIDTH-1:0] m_box_checker_wrd;
    bit [DRAM_DWIDTH-1:0] wrd_to_design;
    int                   repeat_count;
    int                   read_data_ready_low_time;
    int                   hold_read_data_ready_low_time;

    mailbox m_box_checker   = new();
    mailbox m_box_to_design = new();

    int dram_word_count_sent  = 0;
    int dma_word_count_actual = 0;
    bit done                  = 0;

    read_data_ready_low_time      = $urandom_range(0, 8);
    hold_read_data_ready_low_time = $urandom_range(1, 10);

    if (random) begin
      address   = Rand#(AWIDTH)::rand_bit();
      count_int = Rand#(CWIDTH)::rand_bit();
    end else begin
      address   = 1;
      count_int = count;
    end

    repeat_count = $ceil(real'(count_int) / SOURCE_WRD_PER_DRAM_WRD);

    read_gen_data(count_int, random, m_box_checker, m_box_to_design);
    $display("\nCurrent count               = %-d", count_int);
    $display("Current data_valid_low_time = %-d", underflow);

    dma.read_init(count_int, address);
    fork
      begin
        // Load data.
        repeat (repeat_count) begin
          m_box_to_design.get(wrd_to_design);
          dram.send_read(wrd_to_design);
          dram_word_count_sent = dram_word_count_sent + 1;
          while (!dram.dram_rd_data_ready) @(posedge clk);
          if (underflow && dram_word_count_sent < repeat_count) begin
            // Don't call this method if it's the last word. The data will
            // already be declared invalid.
            dram.invalid_data(underflow);
          end
        end
        dram.dram_rd_data_valid = 0;
        do @(posedge clk); while (dma_word_count_actual != count_int);
        done = 1;
      end
      begin
        if (source_underflow) begin
          repeat (repeat_count) begin
            repeat (read_data_ready_low_time) @(posedge clk);
            dma.read_data_ready = 0;
            repeat (hold_read_data_ready_low_time) @(posedge clk);
            dma.read_data_ready = 1;
          end
        end
      end
      begin
        // Data integrity check.
        do begin
          @(negedge clk);
          if (dma.read_data_valid && dma.read_data_ready) begin
            dma_word_count_actual = dma_word_count_actual + 1;
            void'(m_box_checker.try_get(m_box_checker_wrd));
            $display("ACTUAL   read data = %h", dma.read_data);
            $display("EXPECTED read data = %h", m_box_checker_wrd);
            `ASSERT_ERROR(m_box_checker_wrd == dma.read_data,
            "Output read data does not match input read data");
          end
        end while (!done);
      end
    join
  endtask: read

  task automatic read_underflow_multiple ();
  // Test underflow reads and underflow random reads. This tests both underflow
  // from the DRAM/interconnect, and underflow from the DMA source.
  //
    int data_valid_low_time;
    int count;

    for (data_valid_low_time = 1; data_valid_low_time <= 9; data_valid_low_time++) begin
      for (count = 1; count <= 32; count++) begin
        // Regular reads at a given data_valid_low_time.
        read(count, 0, data_valid_low_time, 1);
      end
      // Random reads at a given data_valid_low_time.
      read(0, 1, data_valid_low_time, 1);
    end
  endtask

  task automatic read_multiple ();
  // Test regular reads and random reads.
  //
    int count;

    // Regular reads at a specific count.
    for (count = 1; count <= 32; count++) begin
      read(count, 0, 0, 0);
    end
    // 10 random reads.
    repeat (10) read(0, 1, 0, 0);
  endtask

  initial begin
    string tb_name;

    // Generate a string for the name of this testbench.
    tb_name = $sformatf( {
      "native_dma_interconnect_tb\n",
      "AWIDTH      = %02d, DWIDTH      = %04d\n",
      "DRAM_AWIDTH = %02d, DRAM_DWIDTH = %04d\n",
      "CWIDTH      = %02d" },
      AWIDTH, DWIDTH, DRAM_AWIDTH, DRAM_DWIDTH, CWIDTH
    );

    // Initialize the test exec object for this testbench.
    test.start_tb(tb_name);
    clk_gen.start();

    dma.reset_all();
    dram.reset_all();

    dram.init();

    test.start_test("Test regularflow writes", 10ms);
    write_multiple();
    test.end_test();

    test.start_test("Test underflow writes", 10ms);
    write_underflow_multiple();
    test.end_test();

    test.start_test("Test reads", 10ms);
      read_multiple();
    test.end_test();

    test.start_test("Test underflow reads", 10ms);
      read_underflow_multiple();
    test.end_test();

    clk_gen.kill();
    test.end_tb();
  end

endmodule: native_dma_master_tb
