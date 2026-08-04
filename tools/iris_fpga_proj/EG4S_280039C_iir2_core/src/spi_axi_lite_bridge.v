`timescale 1ns / 1ps

// SPI-mode-0 to AXI4-Lite bridge for the TD/C2000 board.
// SPI command word: bit15=1 read / 0 write, bits14:8=16-bit register index.
// The following 16-bit word is data. Even/odd register indices address the
// low/high half of one 32-bit AXI word and generate the matching WSTRB.
module spi_axi_lite_bridge #(
    parameter integer AXI_ADDR_WIDTH = 12
) (
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      spi_cs_n,
    input  wire                      spi_sclk,
    input  wire                      spi_mosi,
    output wire                      spi_miso,

    output reg [AXI_ADDR_WIDTH-1:0]  m_axi_awaddr,
    output reg                       m_axi_awvalid,
    input  wire                      m_axi_awready,
    output reg [31:0]                m_axi_wdata,
    output reg [3:0]                 m_axi_wstrb,
    output reg                       m_axi_wvalid,
    input  wire                      m_axi_wready,
    input  wire [1:0]                m_axi_bresp,
    input  wire                      m_axi_bvalid,
    output wire                      m_axi_bready,
    output reg [AXI_ADDR_WIDTH-1:0]  m_axi_araddr,
    output reg                       m_axi_arvalid,
    input  wire                      m_axi_arready,
    input  wire [31:0]               m_axi_rdata,
    input  wire [1:0]                m_axi_rresp,
    input  wire                      m_axi_rvalid,
    output wire                      m_axi_rready,

    output reg                       write_complete_pulse,
    output reg                       bridge_error
);
    reg [2:0] sclk_sync;
    reg [2:0] cs_sync;
    reg [1:0] mosi_sync;
    reg [4:0] bit_count;
    reg [15:0] rx_shift;
    reg [15:0] tx_shift;
    reg data_phase;
    reg command_read;
    reg command_half;
    reg [6:0] command_address;
    reg [15:0] read_halfword;
    reg read_data_ready;

    wire sclk_rise = (sclk_sync[2:1] == 2'b01);
    wire sclk_fall = (sclk_sync[2:1] == 2'b10);
    wire cs_active = !cs_sync[1];
    wire [15:0] received_word = {rx_shift[14:0], mosi_sync[1]};

    assign spi_miso    = cs_active ? tx_shift[15] : 1'bz;
    assign m_axi_bready = 1'b1;
    assign m_axi_rready = 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync <= 3'b000;
            cs_sync   <= 3'b111;
            mosi_sync <= 2'b00;
        end else begin
            sclk_sync <= {sclk_sync[1:0], spi_sclk};
            cs_sync   <= {cs_sync[1:0], spi_cs_n};
            mosi_sync <= {mosi_sync[0], spi_mosi};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_count       <= 5'd0;
            rx_shift        <= 16'd0;
            tx_shift        <= 16'd0;
            data_phase      <= 1'b0;
            command_read    <= 1'b0;
            command_half    <= 1'b0;
            command_address <= 7'd0;
            read_halfword   <= 16'd0;
            read_data_ready <= 1'b0;
            m_axi_awaddr    <= {AXI_ADDR_WIDTH{1'b0}};
            m_axi_awvalid   <= 1'b0;
            m_axi_wdata     <= 32'd0;
            m_axi_wstrb     <= 4'd0;
            m_axi_wvalid    <= 1'b0;
            m_axi_araddr    <= {AXI_ADDR_WIDTH{1'b0}};
            m_axi_arvalid   <= 1'b0;
            write_complete_pulse <= 1'b0;
            bridge_error    <= 1'b0;
        end else begin
            write_complete_pulse <= 1'b0;

            if (m_axi_awvalid && m_axi_awready)
                m_axi_awvalid <= 1'b0;
            if (m_axi_wvalid && m_axi_wready)
                m_axi_wvalid <= 1'b0;
            if (m_axi_arvalid && m_axi_arready)
                m_axi_arvalid <= 1'b0;
            if (m_axi_bvalid) begin
                write_complete_pulse <= 1'b1;
                if (m_axi_bresp != 2'b00)
                    bridge_error <= 1'b1;
            end
            if (m_axi_rvalid) begin
                read_halfword <= command_half ? m_axi_rdata[31:16] : m_axi_rdata[15:0];
                read_data_ready <= 1'b1;
                if (m_axi_rresp != 2'b00)
                    bridge_error <= 1'b1;
            end

            if (!cs_active) begin
                bit_count  <= 5'd0;
                data_phase <= 1'b0;
                tx_shift   <= 16'd0;
            end else begin
                if (sclk_rise) begin
                    rx_shift <= received_word;
                    if (bit_count == 5'd15) begin
                        bit_count <= 5'd0;
                        if (!data_phase) begin
                            data_phase      <= 1'b1;
                            command_read    <= received_word[15];
                            command_address <= received_word[14:8];
                            command_half    <= received_word[8];
                            read_data_ready <= 1'b0;
                            if (received_word[15]) begin
                                if (!m_axi_arvalid) begin
                                    m_axi_araddr  <= {{(AXI_ADDR_WIDTH-8){1'b0}},
                                                      received_word[14:9], 2'b00};
                                    m_axi_arvalid <= 1'b1;
                                end else begin
                                    bridge_error <= 1'b1;
                                end
                            end
                        end else begin
                            data_phase <= 1'b0;
                            if (!command_read) begin
                                if (!m_axi_awvalid && !m_axi_wvalid) begin
                                    m_axi_awaddr  <= {{(AXI_ADDR_WIDTH-8){1'b0}},
                                                      command_address[6:1], 2'b00};
                                    m_axi_awvalid <= 1'b1;
                                    if (command_address[0]) begin
                                        m_axi_wdata <= {received_word, 16'd0};
                                        m_axi_wstrb <= 4'b1100;
                                    end else begin
                                        m_axi_wdata <= {16'd0, received_word};
                                        m_axi_wstrb <= 4'b0011;
                                    end
                                    m_axi_wvalid <= 1'b1;
                                end else begin
                                    bridge_error <= 1'b1;
                                end
                            end
                        end
                    end else begin
                        bit_count <= bit_count + 1'b1;
                    end
                end

                if (sclk_fall) begin
                    if (data_phase && bit_count == 0) begin
                        if (command_read && read_data_ready)
                            tx_shift <= read_halfword;
                        else
                            tx_shift <= 16'd0;
                    end else begin
                        tx_shift <= {tx_shift[14:0], 1'b0};
                    end
                end
            end
        end
    end
endmodule
