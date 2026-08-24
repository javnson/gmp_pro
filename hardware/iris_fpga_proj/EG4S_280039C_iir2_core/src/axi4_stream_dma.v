`timescale 1ns / 1ps

// Register-only AXI4 memory-to-stream / stream-to-memory DMA.
// Bulk samples live in external memory; no payload BRAM is inferred.  Read and
// write channels operate concurrently and use INCR bursts.
module axi4_stream_dma #(
    parameter integer ADDR_WIDTH = 32,
    parameter integer DATA_WIDTH = 32,
    parameter integer ID_WIDTH = 1,
    parameter integer MAX_BURST_LEN = 16
) (
    input  wire                       clk,
    input  wire                       rst_n,
    input  wire                       start,
    input  wire [ADDR_WIDTH-1:0]      read_base_addr,
    input  wire [ADDR_WIDTH-1:0]      write_base_addr,
    input  wire [31:0]                sample_count,
    output reg                        read_busy,
    output reg                        write_busy,
    output reg                        done,
    output reg                        error,

    output wire [DATA_WIDTH-1:0]      m_axis_tdata,
    output wire                       m_axis_tvalid,
    input  wire                       m_axis_tready,
    output wire                       m_axis_tlast,
    input  wire [DATA_WIDTH-1:0]      s_axis_tdata,
    input  wire                       s_axis_tvalid,
    output wire                       s_axis_tready,
    input  wire                       s_axis_tlast,

    output wire [ID_WIDTH-1:0]        m_axi_arid,
    output reg  [ADDR_WIDTH-1:0]      m_axi_araddr,
    output reg  [7:0]                 m_axi_arlen,
    output wire [2:0]                 m_axi_arsize,
    output wire [1:0]                 m_axi_arburst,
    output wire                       m_axi_arlock,
    output wire [3:0]                 m_axi_arcache,
    output wire [2:0]                 m_axi_arprot,
    output wire [3:0]                 m_axi_arqos,
    output reg                        m_axi_arvalid,
    input  wire                       m_axi_arready,
    input  wire [ID_WIDTH-1:0]        m_axi_rid,
    input  wire [DATA_WIDTH-1:0]      m_axi_rdata,
    input  wire [1:0]                 m_axi_rresp,
    input  wire                       m_axi_rlast,
    input  wire                       m_axi_rvalid,
    output wire                       m_axi_rready,

    output wire [ID_WIDTH-1:0]        m_axi_awid,
    output reg  [ADDR_WIDTH-1:0]      m_axi_awaddr,
    output reg  [7:0]                 m_axi_awlen,
    output wire [2:0]                 m_axi_awsize,
    output wire [1:0]                 m_axi_awburst,
    output wire                       m_axi_awlock,
    output wire [3:0]                 m_axi_awcache,
    output wire [2:0]                 m_axi_awprot,
    output wire [3:0]                 m_axi_awqos,
    output reg                        m_axi_awvalid,
    input  wire                       m_axi_awready,
    output wire [DATA_WIDTH-1:0]      m_axi_wdata,
    output wire [DATA_WIDTH/8-1:0]    m_axi_wstrb,
    output wire                       m_axi_wlast,
    output wire                       m_axi_wvalid,
    input  wire                       m_axi_wready,
    input  wire [ID_WIDTH-1:0]        m_axi_bid,
    input  wire [1:0]                 m_axi_bresp,
    input  wire                       m_axi_bvalid,
    output wire                       m_axi_bready
);
    localparam integer BYTES_PER_BEAT = DATA_WIDTH / 8;
    localparam integer AXI_SIZE = (BYTES_PER_BEAT == 8) ? 3 :
                                  (BYTES_PER_BEAT == 4) ? 2 :
                                  (BYTES_PER_BEAT == 2) ? 1 : 0;

    reg [31:0] read_remaining;
    reg [31:0] write_remaining;
    reg [8:0]  read_beats_left;
    reg [8:0]  write_beats_left;
    reg        read_burst_active;
    reg        write_burst_active;
    reg        write_response_pending;
    reg        write_final_burst;
    reg        read_done_latched;
    reg        write_done_latched;

    wire [31:0] read_burst_cap = (read_remaining > MAX_BURST_LEN) ?
                                 MAX_BURST_LEN : read_remaining;
    wire [31:0] write_burst_cap = (write_remaining > MAX_BURST_LEN) ?
                                  MAX_BURST_LEN : write_remaining;
    wire [31:0] read_boundary_beats =
        (13'd4096 - {1'b0,m_axi_araddr[11:0]}) >> AXI_SIZE;
    wire [31:0] write_boundary_beats =
        (13'd4096 - {1'b0,m_axi_awaddr[11:0]}) >> AXI_SIZE;
    wire [31:0] next_read_burst = (read_burst_cap > read_boundary_beats) ?
                                  read_boundary_beats : read_burst_cap;
    wire [31:0] next_write_burst = (write_burst_cap > write_boundary_beats) ?
                                   write_boundary_beats : write_burst_cap;

    assign m_axi_arsize  = AXI_SIZE[2:0];
    assign m_axi_awsize  = AXI_SIZE[2:0];
    assign m_axi_arid    = {ID_WIDTH{1'b0}};
    assign m_axi_awid    = {ID_WIDTH{1'b0}};
    assign m_axi_arburst = 2'b01;
    assign m_axi_awburst = 2'b01;
    assign m_axi_arlock  = 1'b0;
    assign m_axi_awlock  = 1'b0;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_awprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_awqos   = 4'b0000;
    assign m_axis_tdata  = m_axi_rdata;
    assign m_axis_tvalid = read_burst_active && m_axi_rvalid;
    assign m_axis_tlast  = (read_remaining == 1) && m_axis_tvalid;
    assign m_axi_rready  = read_burst_active && m_axis_tready;

    assign m_axi_wdata   = s_axis_tdata;
    assign m_axi_wstrb   = {DATA_WIDTH/8{1'b1}};
    assign m_axi_wvalid  = write_burst_active && s_axis_tvalid;
    assign m_axi_wlast   = write_burst_active && (write_beats_left == 1);
    assign s_axis_tready = write_burst_active && m_axi_wready;
    assign m_axi_bready  = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_busy             <= 1'b0;
            write_busy            <= 1'b0;
            done                  <= 1'b0;
            error                 <= 1'b0;
            read_remaining        <= 32'd0;
            write_remaining       <= 32'd0;
            read_beats_left       <= 9'd0;
            write_beats_left      <= 9'd0;
            read_burst_active     <= 1'b0;
            write_burst_active    <= 1'b0;
            write_response_pending<= 1'b0;
            write_final_burst     <= 1'b0;
            read_done_latched     <= 1'b0;
            write_done_latched    <= 1'b0;
            m_axi_araddr          <= {ADDR_WIDTH{1'b0}};
            m_axi_arlen           <= 8'd0;
            m_axi_arvalid         <= 1'b0;
            m_axi_awaddr          <= {ADDR_WIDTH{1'b0}};
            m_axi_awlen           <= 8'd0;
            m_axi_awvalid         <= 1'b0;
        end else begin
            done <= 1'b0;

            if (start) begin
                if (!read_busy && !write_busy) begin
                    if ((read_base_addr[1:0] != 0) ||
                        (write_base_addr[1:0] != 0)) begin
                        error <= 1'b1;
                        done <= 1'b1;
                    end else begin
                        error                  <= 1'b0;
                        read_remaining         <= sample_count;
                        write_remaining        <= sample_count;
                        m_axi_araddr            <= read_base_addr;
                        m_axi_awaddr            <= write_base_addr;
                        read_busy               <= (sample_count != 0);
                        write_busy              <= (sample_count != 0);
                        read_done_latched       <= (sample_count == 0);
                        write_done_latched      <= (sample_count == 0);
                        if (sample_count == 0)
                            done <= 1'b1;
                    end
                end else begin
                    error <= 1'b1;
                end
            end

            // Read address and data channels.
            if (read_busy && !read_burst_active && !m_axi_arvalid &&
                read_remaining != 0) begin
                m_axi_arlen   <= next_read_burst[7:0] - 1'b1;
                m_axi_arvalid <= 1'b1;
            end
            if (m_axi_arvalid && m_axi_arready) begin
                m_axi_arvalid     <= 1'b0;
                read_burst_active <= 1'b1;
                read_beats_left   <= {1'b0,m_axi_arlen} + 1'b1;
                m_axi_araddr      <= m_axi_araddr
                                   + (({24'd0,m_axi_arlen} + 1'b1) * BYTES_PER_BEAT);
            end
            if (m_axi_rvalid && m_axi_rready) begin
                if (m_axi_rresp != 2'b00)
                    error <= 1'b1;
                if (m_axi_rlast != (read_beats_left == 1))
                    error <= 1'b1;
                read_remaining  <= read_remaining - 1'b1;
                read_beats_left <= read_beats_left - 1'b1;
                if (read_beats_left == 1) begin
                    read_burst_active <= 1'b0;
                    if (read_remaining == 1) begin
                        read_busy         <= 1'b0;
                        read_done_latched <= 1'b1;
                    end
                end
            end

            // Write address, data, and response channels.
            if (write_busy && !write_burst_active && !write_response_pending &&
                !m_axi_awvalid && write_remaining != 0) begin
                m_axi_awlen   <= next_write_burst[7:0] - 1'b1;
                m_axi_awvalid <= 1'b1;
            end
            if (m_axi_awvalid && m_axi_awready) begin
                m_axi_awvalid      <= 1'b0;
                write_burst_active <= 1'b1;
                write_beats_left   <= {1'b0,m_axi_awlen} + 1'b1;
                write_final_burst  <= (write_remaining <= MAX_BURST_LEN);
                m_axi_awaddr       <= m_axi_awaddr
                                    + (({24'd0,m_axi_awlen} + 1'b1) * BYTES_PER_BEAT);
            end
            if (m_axi_wvalid && m_axi_wready) begin
                if (s_axis_tlast != (write_remaining == 1))
                    error <= 1'b1;
                write_remaining  <= write_remaining - 1'b1;
                write_beats_left <= write_beats_left - 1'b1;
                if (write_beats_left == 1) begin
                    write_burst_active     <= 1'b0;
                    write_response_pending <= 1'b1;
                end
            end
            if (m_axi_bvalid && m_axi_bready) begin
                if (!write_response_pending || m_axi_bresp != 2'b00)
                    error <= 1'b1;
                write_response_pending <= 1'b0;
                if (write_final_burst) begin
                    write_busy         <= 1'b0;
                    write_done_latched <= 1'b1;
                end
            end

            if (read_done_latched && write_done_latched &&
                !read_busy && !write_busy) begin
                done               <= 1'b1;
                read_done_latched  <= 1'b0;
                write_done_latched <= 1'b0;
            end
        end
    end
endmodule
