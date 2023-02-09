//
// Copyright 2022 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: native_dma_master
//
// Description:
//
//   This module translates DMA source signals to native DRAM signals to be
//   passed to an interconnect that can communicate with the memory controller.
//   This design assumes dram_rdy and dram_wdf_rdy will only ever be deasserted
//   after a full DRAM_DWIDTH word from the interconnect.
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
// Signals:
//
//   See port list.
//

// TODO: Add command path interleaving. Currently the TB only calls writes and
//       reads sequentially, not concurrently.
// TODO: Deassertions of write_data_valid not fully accounted for.

module native_dma_master #(
  parameter AWIDTH       = 32,
  parameter DWIDTH       = 64,
  parameter DRAM_AWIDTH  = 32,
  parameter DRAM_DWIDTH  = 512,
  parameter CWIDTH       = 8,
  parameter BURST_LENGTH = 8
) (

  input  wire                        clk,
  input  wire                        rst,

  // Between DMA SOURCE and MASTER
  // Write transaction interface

  input  wire           [AWIDTH-1:0] write_addr,          // address to write to
  input  wire           [CWIDTH-1:0] write_count,         // how many write transactions of DWIDTH
  input  wire                        write_ctrl_valid,    // DMA source wants to start write
  output reg                         write_ctrl_ready,    // master able to start write
  input  wire           [DWIDTH-1:0] write_data,          // from DMA source
  input  wire                        write_data_valid,    // sent data from DMA source is valid
  output reg                         write_data_ready,    // ready to write to interconnect fifo

  // Read transaction interface

  input  wire           [AWIDTH-1:0] read_addr,           // address to read from
  input  wire           [CWIDTH-1:0] read_count,          // how many read transactions of DWIDTH
  input  wire                        read_ctrl_valid,     // DMA source wants to start read
  output reg                         read_ctrl_ready,     // master able to start read
  output reg            [DWIDTH-1:0] read_data,           // to DMA source
  output reg                         read_data_valid,     // data sent to DMA source is valid
  input  wire                        read_data_ready,     // DMA source ready to read

  // Between MASTER and INTERCONNECT
  // Both Read & Write

  input  wire                        init_calib_complete, // DRAM done calibrating
  output reg       [DRAM_AWIDTH-1:0] dram_addr,           // address for current request
  output reg                   [2:0] dram_cmd,            // write: 000, read: 001, wr_bytes: 011
  output reg                         dram_en,             // strobe for dram_addr[] and dram_cmd[]
  input  wire                        dram_rdy,            // ready to accept commands

  // Write

  // Packet Structure:
  // dram_cmd + dram_addr + dram_wdf_data + dram_wdf_mask +
  // dram_wdf_wren + dram_wdf_end + dram_en
  output reg [3+DRAM_AWIDTH+DRAM_DWIDTH+(DRAM_DWIDTH/8)+3-1:0] write_packet,

  output reg                         dram_wdf_end,        // last cycle of data to be written to DRAM
  output reg                         dram_wdf_wren,       // strobe for dram_wdf_data[]
  output reg       [DRAM_DWIDTH-1:0] dram_wdf_data,       // send data to interconnect
  output reg   [(DRAM_DWIDTH/8)-1:0] dram_wdf_mask,       // data mask for dram_wdf_data; masks per byte
  input  wire                        dram_wdf_rdy,        // ready to write data into DRAM

  // Read

  input  wire      [DRAM_DWIDTH-1:0] dram_rd_data,        // receive data from interconnect
  input  wire                        dram_rd_data_valid,  // valid signal for dram_rd_data[]
  output reg                         dram_rd_data_ready   // ready for new data from interconnect
);

  localparam SOURCE_WRD_PER_DRAM_WRD = (DRAM_DWIDTH / DWIDTH);
  localparam logic [2:0] WRITE = 3'b000, READ = 3'b001, WR_BYTES = 3'b011; // Command encoding

  // Write SM

  enum { W_CALBR, W_IDLE, W_START, W_WRITE } w_state;

  reg  [$clog2(SOURCE_WRD_PER_DRAM_WRD)+1:0] w_word_completion_counter; // counter used to pack data
  reg                           [CWIDTH-1:0] w_count_cache;             // count_cache for write
  reg                      [DRAM_AWIDTH-1:0] w_addr_cache;              // cache for write address
  reg  [$clog2(SOURCE_WRD_PER_DRAM_WRD)+1:0] w_mask_count;              // count used to know which bits to mask
  reg                      [DRAM_DWIDTH-1:0] write_data_cache;

  always_ff @(posedge clk) begin

    if (rst) begin
      w_state <= W_CALBR;
    end else begin

      case (w_state)
        W_CALBR: begin
          dram_wdf_wren    <= 1'b0;
          dram_wdf_end     <= 1'b0;
          dram_en          <= 1'b0;
          write_ctrl_ready <= 1'b0;
          write_data_ready <= 1'b0;
          dram_wdf_wren    <= 1'b0;
          dram_wdf_end     <= 1'b0;
          w_count_cache    <= 1'b0;
          if (init_calib_complete) begin
            w_state <= W_IDLE;
          end
        end

        W_IDLE: begin
          write_ctrl_ready <= 1'b1;
          write_data_cache <= {DRAM_DWIDTH{1'b0}};
          if (write_ctrl_valid) begin
            write_ctrl_ready <= 1'b0;
            w_addr_cache     <= write_addr;
            w_count_cache    <= write_count;
            dram_cmd         <= WRITE;
            w_state <= W_START;
          end
        end

        W_START: begin
          write_data_ready <= 1'b1;
          // Wait for write_data_valid.
          if (write_data_valid) begin
            // Process first word and start unpacking it.
            w_addr_cache     <= w_addr_cache + BURST_LENGTH;
            dram_addr        <= w_addr_cache;
            write_data_cache <= {write_data_cache[DRAM_DWIDTH-DWIDTH-1:0], write_data};
            w_count_cache    <= w_count_cache - 1;
            dram_wdf_mask    <= {{(DRAM_DWIDTH/8/SOURCE_WRD_PER_DRAM_WRD)-1{8'hFF}},8'h00};
            if (w_count_cache >= SOURCE_WRD_PER_DRAM_WRD) begin
              w_word_completion_counter <= SOURCE_WRD_PER_DRAM_WRD - 1;
              w_mask_count <= SOURCE_WRD_PER_DRAM_WRD;
            end else begin
              w_word_completion_counter <= w_count_cache - 1;
              w_mask_count <= w_count_cache;
            end
            w_state <= W_WRITE;
          end
        end

        W_WRITE: begin
          if (dram_rdy && dram_wdf_rdy) begin
            dram_wdf_wren    <= 1'b0;
            dram_wdf_end     <= 1'b0;
            dram_en          <= 1'b0;
            if (w_count_cache > 0) begin
            // If write_data_ready was deasserted, but dram_rdy && dram_wdf_rdy
            // are back high, and w_count_cache is still greater than 0, we
            // need to reassert write_data_ready to continue getting data.
              write_data_ready <= 1'b1;
            end
            if (write_data_valid) begin
            w_count_cache    <= w_count_cache - 1;
            write_data_cache <= {write_data_cache[DRAM_DWIDTH-DWIDTH-1:0], write_data};
            dram_wdf_mask    <= {dram_wdf_mask[0+:(DRAM_DWIDTH-DWIDTH)/8],{(DWIDTH/8){1'b0}}};
              if (w_word_completion_counter == 0) begin
                // Finished packing current word, update word_completion_counter.
                if (w_count_cache >= SOURCE_WRD_PER_DRAM_WRD) begin
                  w_word_completion_counter <= SOURCE_WRD_PER_DRAM_WRD - 1;
                  w_mask_count <= SOURCE_WRD_PER_DRAM_WRD;
                end else begin
                  w_word_completion_counter <= w_count_cache - 1;
                  w_mask_count <= w_count_cache;
                end
              end else begin
                // Not finished packing current word.
                w_word_completion_counter <= w_word_completion_counter - 1;
              end
            end else begin
              if (w_word_completion_counter > SOURCE_WRD_PER_DRAM_WRD - 1) begin
              // Only on the last cycle of a write transaction,
              // w_word_completion_counter gets decremented while it's at 0,
              // meaning it rolls over to the max value it can hold. If this
              // happens we know we need to go back to W_IDLE, and that we're
              // done.
                write_ctrl_ready <= 1'b1;
                w_state          <= W_IDLE;
              end else begin
                write_data_ready          <= 1'b0;
                w_word_completion_counter <= w_word_completion_counter - 1;
              end

              if (w_count_cache == 0) begin
                dram_wdf_mask <= {dram_wdf_mask[0+:(DRAM_DWIDTH-DWIDTH)/8],{(DWIDTH/8){1'b0}}};
              end
            end
          end
        end

        default:
          w_state <= W_CALBR;
      endcase

      // TODO: This adds 1 clock cycle of latency every time dram_rdy or
      // dram_wdf_rdy gets deasserted in a write transaction.
      if (w_word_completion_counter == 0) begin
        if (dram_rdy && dram_wdf_rdy) begin
          dram_wdf_wren <= 1'b1;
          dram_wdf_end  <= 1'b1;
          dram_en       <= 1'b1;
          dram_wdf_data <= write_data_cache;
          write_packet  <= {dram_cmd,dram_addr,write_data_cache,dram_wdf_mask,dram_wdf_wren,dram_wdf_end,dram_en};
        end else begin
          // We have a finished word and are waiting, tell the DMA
          // source not to send data.
          write_data_ready <= 1'b0;
        end
      end

      if (dram_rdy && dram_wdf_wren && dram_wdf_end && dram_en) begin
        dram_wdf_mask <= {{(DRAM_DWIDTH/8/SOURCE_WRD_PER_DRAM_WRD)-1{8'hFF}}, 8'h00};
        w_addr_cache  <= w_addr_cache + BURST_LENGTH;
        dram_addr     <= w_addr_cache;
      end

    end
  end

  // Read Data From FIFO SM

  enum { R_CALBR, R_IDLE, R_START, R_READ, R_ERR_HANDLER } r_state;

  reg                                       read_data_cache_valid;
  reg [$clog2(SOURCE_WRD_PER_DRAM_WRD)+1:0] r_word_completion_counter; // counter used to unpack data from FIFO
  reg                          [CWIDTH-1:0] r_count_cache;             // count_cache for read and write
  reg                     [DRAM_AWIDTH-1:0] r_addr_cache;              // cache for read address
  reg                     [DRAM_DWIDTH-1:0] read_data_cache;
  reg                     [DRAM_DWIDTH-1:0] extra_read_data_cache;
  reg                                       r_addr_ctrl;
  reg                          [CWIDTH-1:0] a_count_cache;             // count_cache for address when reading

  always_ff @(posedge clk) begin

    if (rst) begin
      r_state <= R_CALBR;
    end else begin

      case (r_state)
        R_CALBR: begin
          read_ctrl_ready    <= 1'b0;
          read_data_valid    <= 1'b0;
          dram_rd_data_ready <= 1'b0;
          r_count_cache      <= 1'b0;
          a_count_cache      <= 1'b0;
          if (init_calib_complete) begin
            r_state <= R_IDLE;
          end
        end

        R_IDLE: begin
          // Wait to start a read transaction.
          read_ctrl_ready <= 1'b1;
          if (read_ctrl_valid) begin
            read_ctrl_ready    <= 1'b0;
            r_count_cache      <= read_count;
            a_count_cache      <= read_count;
            dram_addr          <= read_addr;
            dram_en            <= 1'b1;
            r_addr_ctrl        <= 1'b1;
            dram_rd_data_ready <= 1'b1;
            dram_cmd           <= READ;
            r_state            <= R_START;
          end
        end

        R_START: begin
          // Set counter depending on how many words there are.
            if (r_count_cache >= SOURCE_WRD_PER_DRAM_WRD) begin
              r_word_completion_counter <= SOURCE_WRD_PER_DRAM_WRD;
            end else begin
              r_word_completion_counter <= r_count_cache;
            end
          // Started a read transaction, wait for data from interconnect.
          if (dram_rd_data_valid) begin
            // Design will only assert dram_rd_data_ready if there's valid data.
            dram_rd_data_ready    <= 1'b0;
            read_data_cache_valid <= 1'b1;
            read_data_cache       <= dram_rd_data;
            r_state               <= R_READ;
          end
        end

        R_READ: begin
          // dram_rd_data_ready only high for a single clock cycle after the
          // design grabs a new word.
          if (read_data_ready) begin
            if (dram_rd_data_ready && dram_rd_data_valid) begin
              dram_rd_data_ready    <= 1'b0;
              read_data_cache_valid <= 1'b1;
              read_data_cache       <= dram_rd_data;
              if (r_count_cache >= SOURCE_WRD_PER_DRAM_WRD + 1) begin
                r_word_completion_counter <= SOURCE_WRD_PER_DRAM_WRD;
              end else begin
                if (!read_data_cache_valid) begin
                  r_word_completion_counter <= r_count_cache;
                end else begin
                  r_word_completion_counter <= r_count_cache - 1;
                end
              end
            end

            if (r_word_completion_counter == 2 && r_count_cache >= 3) begin
              // Grab a new word if we need one.
              dram_rd_data_ready <= 1'b1;
            end else if (r_word_completion_counter == 1
            && !(dram_rd_data_ready && dram_rd_data_valid)) begin
              read_data_cache_valid <= 1'b0;
            end else if (r_word_completion_counter == 0) begin
              read_data_valid <= 1'b0;
            end

            if (r_count_cache == 0) begin
              read_data_valid       <= 1'b0;
              read_data_cache_valid <= 1'b0;
              r_state               <= R_IDLE;
            end

            if (read_data_cache_valid) begin
              read_data_valid <= 1'b1;
              r_count_cache   <= r_count_cache - 1;
              read_data       <= read_data_cache[DWIDTH*(r_word_completion_counter-1)+:DWIDTH];
              if (!(dram_rd_data_ready && dram_rd_data_valid)) begin
                // If dram_rd_data_ready is high r_word_completion_counter is
                // waiting to be reassigned, not decremented.
                r_word_completion_counter <= r_word_completion_counter - 1;
              end
            end
          end else begin
            if (dram_rd_data_ready && dram_rd_data_valid) begin
              // This only happens if read_data_ready gets deasserted the same
              // posedge that ready_data_ready goes high.
              dram_rd_data_ready    <= 1'b0;
              read_data_cache_valid <= 1'b1;
              extra_read_data_cache <= dram_rd_data;
              r_state               <= R_ERR_HANDLER;
              if (r_word_completion_counter == 0) begin
                if (r_count_cache >= SOURCE_WRD_PER_DRAM_WRD + 1) begin
                  r_word_completion_counter <= SOURCE_WRD_PER_DRAM_WRD;
                end else begin
                  if (!read_data_cache_valid) begin
                    r_word_completion_counter <= r_count_cache;
                  end else begin
                    r_word_completion_counter <= r_count_cache - 1;
                  end
                end
              end
            end
          end
        end

        R_ERR_HANDLER: begin
          if (read_data_ready) begin
            read_data_cache       <= extra_read_data_cache;
            read_data_cache_valid <= 1'b1;
            r_count_cache         <= r_count_cache - 1;
            r_state               <= R_READ;
            if (r_word_completion_counter == r_count_cache) begin
              read_data             <= extra_read_data_cache[DWIDTH*(r_word_completion_counter-1)+:DWIDTH];
            end else begin
              read_data             <= read_data_cache[DWIDTH*(r_word_completion_counter-1)+:DWIDTH];
            end
            // Reassign r_word_completion_counter as when we got to this state
            // we were on our last state.
            if (r_count_cache >= SOURCE_WRD_PER_DRAM_WRD + 1) begin
              r_word_completion_counter <= SOURCE_WRD_PER_DRAM_WRD;
            end else begin
              if (!read_data_cache_valid) begin
                r_word_completion_counter <= r_count_cache;
              end else begin
                r_word_completion_counter <= r_count_cache - 1;
              end
            end
          end
        end

        default:
          r_state <= R_CALBR;
      endcase

    end
  end

  // Read Address Process

  always_ff @(posedge clk) begin
  // Update address until count is depleted.
    if (dram_rdy && r_addr_ctrl) begin
      if (a_count_cache == 1) begin
      // If it's a count of 1 it doesn't need to get incremented.
        dram_en     <= 1'b0;
        r_addr_ctrl <= 1'b0;
      end else begin
        dram_addr     <= dram_addr + BURST_LENGTH;
        a_count_cache <= a_count_cache - 1;
      end
    end
  end

endmodule: native_dma_master
