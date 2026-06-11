`resetall
`timescale 1ns / 1ps
`default_nettype none

/*
 * UltraScale PCIe AXI Master Write Path
 */

module taxi_pcie_us_axi_master_wr # (
    // Force 64 bit address
    parameter logic TLP_FORCE_64_BIT_ADDR = 1'b0
)
(
    input  wire logic         clk,
    input  wire logic         rst,
     /*
     * TLP input (request)
     */
    taxi_axis_if.snk      s_axis_cq,


      /*
     * AXI Master Write output
     */

    taxi_axi_if.wr_mst     m_axi_wr,
   

    /*
     * Status
     */
    output wire logic         stat_err_cor,
    output wire logic         stat_err_uncor

);

    // extract parameters from interface

localparam AXIS_PCIE_DATA_W = s_axis_cq.DATA_W;
localparam AXIS_PCIE_KEEP_W = s_axis_cq.KEEP_W;
localparam AXIS_PCIE_CQ_USER_W = s_axis_cq.USER_W;



localparam AXI_DATA_W = m_axi_wr.DATA_W;
localparam AXI_ADDR_W = m_axi_wr.ADDR_W;
localparam AXI_STRB_W = m_axi_wr.STRB_W;
localparam AXI_MAX_BURST_LEN = m_axi_wr.MAX_BURST_LEN; // maximum burst length in dwords 

localparam AXI_WORD_WIDTH = AXI_STRB_W; // number of bytes in an AXI word
localparam  AXI_WORD_SIZE = AXI_DATA_W/AXI_WORD_WIDTH;
localparam AXI_BURST_SIZE = $clog2(AXI_STRB_W);
localparam AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN*AXI_WORD_WIDTH;

localparam TLP_DATA_WIDTH_BYTES = s_axis_cq.DATA_W/8;
localparam TLP_DATA_WIDTH_DWORDS = s_axis_cq.DATA_W/32;
localparam TLP_SEGS = s_axis_cq.SEGS;
localparam TLP_HDR_W = s_axis_cq.HDR_W;
localparam FUNC_NUM_W = s_axis_cq.FUNC_NUM_W;

localparam OFFSET_WIDTH = $clog2(TLP_DATA_WIDTH_DWORDS);

localparam TAG_W = 10;

localparam CHUNK_W = $clog2(TLP_DATA_W/AXI_DATA_W);

localparam RESP_FIFO_ADDR_W = 5;

// check configuration


if (TLP_SEGS != 1)
        $fatal("Error: TLP segment count must be 1 (instance %m)");

if (TLP_HDR_W != 128) 
        $fatal("Error: TLP segment header width must be 128 (instance %m)");
       

if (AXIS_PCIE_DATA_W != 64 && AXIS_PCIE_DATA_W != 128 && AXIS_PCIE_DATA_W != 256 && AXIS_PCIE_DATA_W != 512)
    $fatal(0, "Error: PCIe interface width must be 64, 128, 256, or 512 (instance %m)");

if (AXIS_PCIE_KEEP_W * 32 != AXIS_PCIE_DATA_W)
    $fatal(0, "Error: PCIe interface requires dword (32-bit) granularity (instance %m)");

if (AXIS_PCIE_DATA_W == 512) begin
    if (AXIS_PCIE_CQ_USER_W != 183)
        $fatal(0, "Error: PCIe CQ tuser width must be 183 (instance %m)");

    if (AXIS_PCIE_CC_USER_W != 81)
        $fatal(0, "Error: PCIe CC tuser width must be 81 (instance %m)");
end else begin
    if (AXIS_PCIE_CQ_USER_W != 85 && AXIS_PCIE_CQ_USER_W != 88)
        $fatal(0, "Error: PCIe CQ tuser width must be 85 or 88 (instance %m)");

    if (AXIS_PCIE_CC_USER_W != 33)
        $fatal(0, "Error: PCIe CC tuser width must be 33 (instance %m)");
end

if (AXI_DATA_W != 32)
    $fatal(0, "Error: AXI interface width must be 32 (instance %m)");

if (AXI_STRB_W * 8 != AXI_DATA_W)
    $fatal(0, "Error: AXI interface requires byte (8-bit) granularity (instance %m)");

   if (MAX_BURST_LEN < 1 || MAX_BURST_LEN > 256)
    $fatal("Error: MAX_BURST_LEN must be between 1 and 256 (instance %m)");
        

typedef enum logic [3:0] {
    REQ_MEM_READ = 4'b0000,
    REQ_MEM_WRITE = 4'b0001,
    REQ_IO_READ = 4'b0010,
    REQ_IO_WRITE = 4'b0011,
    REQ_MEM_FETCH_ADD = 4'b0100,
    REQ_MEM_SWAP = 4'b0101,
    REQ_MEM_CAS = 4'b0110,
    REQ_MEM_READ_LOCKED = 4'b0111,
    REQ_CFG_READ_0 = 4'b1000,
    REQ_CFG_READ_1 = 4'b1001,
    REQ_CFG_WRITE_0 = 4'b1010,
    REQ_CFG_WRITE_1 = 4'b1011,
    REQ_MSG = 4'b1100,
    REQ_MSG_VENDOR = 4'b1101,
    REQ_MSG_ATS = 4'b1110
} req_type_t;

typedef enum logic [2:0] {
    CPL_STATUS_SC  = 3'b000, // successful completion
    CPL_STATUS_UR  = 3'b001, // unsupported request
    CPL_STATUS_CRS = 3'b010, // configuration request retry status
    CPL_STATUS_CA  = 3'b100  // completer abort
} cpl_status_t;

typedef enum logic [1:0] {
    STATE_IDLE = 2'b00,
    STATE_HEADER = 2'b01,
    STATE_TRANSFER = 2'b10,
    STATE_WAIT_END = 2'b11
} state_t;

// datapath control signals
logic transfer_in_save;
logic flush_save;

state_t state_reg = STATE_IDLE, state_next;

logic [AXI_ADDR_W-1:0] axi_addr_reg ='0 , axi_addr_next;
logic [10:0] op_dword_count_reg = '0, op_dword_count_next;
logic [10:0] tr_dword_count_reg = '0, tr_dword_count_next;
logic [12:0] input_cycle_count_reg = '0, input_cycle_count_next;
logic [12:0] output_cycle_count_reg = '0, output_cycle_count_next;
logic input_active_reg = 1'b0, input_active_next;
logic first_cycle_reg = 1'b0, first_cycle_next;
logic last_cycle_reg = 1'b0, last_cycle_next;
logic bubble_cycle_reg = 1'b0, bubble_cycle_next;

logic [3:0] type_reg = 4'd0, type_next;
logic [3:0] first_be_reg = 4'd0, first_be_next;
logic [3:0] last_be_reg = 4'd0, last_be_next;
logic [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next;
logic [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next;

logic [CHUNK_W-1:0] resp_chunk_reg = 0, resp_chunk_next;

// extracted TLP header fields
logic [2:0] s_axis_cq_tlp_hdr_fmt;
logic [4:0] s_axis_cq_tlp_hdr_type;
logic [2:0] s_axis_cq_tlp_hdr_tc;
logic s_axis_cq_tlp_hdr_ln;
logic s_axis_cq_tlp_hdr_th;
logic s_axis_cq_tlp_hdr_td;
logic s_axis_cq_tlp_hdr_ep;
logic [2:0] s_axis_cq_tlp_hdr_attr;
logic [1:0] s_axis_cq_tlp_hdr_at;
logic [10:0] s_axis_cq_tlp_hdr_length;
logic [15:0] s_axis_cq_tlp_hdr_requester_id;
logic [9:0] rx_req_tlp_hdr_tag;
logic [7:0] rx_req_tlp_hdr_last_be;
logic [7:0] rx_req_tlp_hdr_first_be;
logic [63:0] rx_req_tlp_hdr_addr;
logic [1:0] rx_req_tlp_hdr_ph;

// derived TLP header fields
logic [1:0] rx_req_first_be_offset;
logic [1:0] rx_req_last_be_offset;
logic [2:0] rx_req_single_dword_len;

logic [127:0] cpl_tlp_hdr;

logic [RESP_FIFO_ADDR_W+1-1:0] resp_fifo_wr_ptr_reg = '0;
logic [RESP_FIFO_ADDR_W+1-1:0] resp_fifo_rd_ptr_reg = '0, resp_fifo_rd_ptr_next;



logic s_axis_cq_tready_reg = 1'b0, s_axis_cq_tready_next;

logic [AXI_ADDR_W-1:0] m_axi_awaddr_reg = '0, m_axi_awaddr_next;
logic [7:0] m_axi_awlen_reg = 8'd0, m_axi_awlen_next;
logic m_axi_awvalid_reg = 1'b0, m_axi_awvalid_next;

logic [TLP_DATA_WIDTH-1:0] save_axis_tdata_reg = '0;

logic [TLP_DATA_WIDTH-1:0] shift_tlp_data = {s_axis_cq.tdata, save_axis_tdata_reg} >> ((TLP_DATA_WIDTH_DWORDS/4-offset_reg)*32);

logic status_error_uncor_reg = 1'b0, status_error_uncor_next;
logic status_error_cor_reg = 1'b0, status_error_cor_next;


// internal datapath signals
logic  [AXI_DATA_W-1:0]  m_axi_wdata_int;
logic  [AXI_STRB_W-1:0]  m_axi_wstrb_int;
logic                    m_axi_wvalid_int;
logic                    m_axi_wlast_int;
wire                     m_axi_wready_int;

assign s_axis_cq.tready = s_axis_cq_tready_reg;


// initialization of AW Channel signals
assign m_axi_wr.awid = '0;
assign m_axi_wr.awaddr = m_axi_awaddr_reg;
assign m_axi_wr.awlen = m_axi_awlen_reg;
assign m_axi_wr.awsize = $clog2(AXI_STRB_W);
assign m_axi_wr.awburst = 2'b01;
assign m_axi_wr.awlock = 1'b0;
assign m_axi_wr.awcache = 4'b0011;
assign m_axi_wr.awprot = 3'b010;
assign m_axi_wr.awqos = '0;
assign m_axi_wr.awregion = '0;
assign m_axi_wr.awvalid = m_axi_awvalid_reg;

assign m_axi_wr.m_axi_bready = 1'b1;


assign status_error_uncor = status_error_uncor_reg;
assign status_error_cor = status_error_cor_reg;

// Combinational path for FSM and datapath control signals
always_comb begin
    state_next = STATE_IDLE;

    transfer_in_save = 1'b0;

    s_axis_cq_tready_next = 1'b0;

    type_next = type_reg;
    axi_addr_next = axi_addr_reg;
    op_dword_count_next = op_dword_count_reg;
    tr_dword_count_next = tr_dword_count_reg;
    input_cycle_count_next = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next = input_active_reg;
    bubble_cycle_next = bubble_cycle_reg;
    first_cycle_next = first_cycle_reg;
    last_cycle_next = last_cycle_reg;
    first_be_next = first_be_reg;
    last_be_next = last_be_reg;
    offset_next = offset_reg;
    first_cycle_offset_next = first_cycle_offset_reg;
    last_cycle_offset_next = last_cycle_offset_reg;

    m_axi_awaddr_next = m_axi_awaddr_reg;
    m_axi_awlen_next = m_axi_awlen_reg;
    m_axi_awvalid_next = m_axi_awvalid_reg && !m_axi_awready;

    m_axi_wdata_int = shift_axis_tdata;
    m_axi_wstrb_int = '1;
    m_axi_wvalid_int = 1'b0;
    m_axi_wlast_int = 1'b0;

    status_error_uncor_next = 1'b0;
    status_error_cor_next = 1'b0;

    // FSM
    case(state_reg) 
        STATE_IDLE: begin
            // idle state, wait for completion request
            if (AXIS_PCIE_DATA_WIDTH > 64) begin
                s_axis_cq_tready_next = m_axi_wready_int_early && (!m_axi_wr.awvalid || m_axi_wr.awready);

                if (s_axis_cq.tready && s_axis_cq.tvalid) begin
                    transfer_in_save = 1'b1;

                    // header fields s_axis_cq.tdata
                    axi_addr_next = {s_axis_cq.tdata, 2'b00};
                    op_dword_count_next = s_axis_cq.tdata[74:64];
                    type_next = s_axis_cq.tdata[78:75];
                    // tuser fields
                    if (AXIS_PCIE_DATA_WIDTH == 512) begin
                        first_be_next = s_axis_cq.tuser[3:0];
                        last_be_next = s_axis_cq.tuser[11:8];
                    end else begin
                        first_be_next = s_axis_cq.tuser[3:0];
                        last_be_next = s_axis_cq.tuser[7:4];
                    end
                
                    if (op_dword_count_next == 1) begin
                        // use first_be for both byte enables for single DWORD transfers
                        last_be_next = first_be_next;
                    end

                    if (op_dword_count_next <= AXI_MAX_BURST_SIZE/4) begin
                        // packet smaller than max burst size
                        // assumed to not cross 4k boundary, send one request
                        tr_dword_count_next = op_dword_count_next;
                        m_axi_awlen_next = (tr_dword_count_next + axi_addr_next[OFFSET_WIDTH+2-1:2] - 1) >> (AXI_BURST_SIZE-2);
                    end else begin
                        // packet larger than max burst size
                        // assumed to not cross 4k boundary, send one request
                        tr_dword_count_next = AXI_MAX_BURST_SIZE/4 - axi_addr_next[OFFSET_WIDTH+2-1:2];
                        m_axi_awlen_next = (tr_dword_count_next - 1) >> (AXI_BURST_SIZE-2);
                    end

                    m_axi_awaddr_next = axi_addr_next;

                    // required DWORD shift to place first DWORD from the TLP payload into proper position on AXI interface
                    // bubble cycle required if first TLP payload transfer does not fill first AXI transfer
                    if (AXIS_PCIE_DATA_WIDTH >= 256) begin
                        offset_next = axi_addr_next[OFFSET_WIDTH+2-1:2] - 4;
                        bubble_cycle_next = axi_addr_next[OFFSET_WIDTH+2-1:2] < 4;
                    end else begin
                        offset_next = axi_addr_next[OFFSET_WIDTH+2-1:2];
                        bubble_cycle_next = 1'b0;
                    end
                    first_cycle_offset_next = axi_addr_next[OFFSET_WIDTH+2-1:2];
                    first_cycle_next = 1'b1;

                    // number of bus transfers in TLP, DOWRD count plus payload start DWORD offset, divided by bus width in DWORDS
                    if (AXIS_PCIE_DATA_WIDTH >= 256) begin
                        input_cycle_count_next = (tr_dword_count_next + 4 - 1) >> (AXI_BURST_SIZE-2);
                    end else begin
                        input_cycle_count_next = (tr_dword_count_next - 1) >> (AXI_BURST_SIZE-2);
                    end
                    // number of bus transfers to AXI, DWORD count plus DWORD offset, divided by bus width in DWORDS
                    output_cycle_count_next = (tr_dword_count_next + axi_addr_next[OFFSET_WIDTH+2-1:2] - 1) >> (AXI_BURST_SIZE-2);
                    last_cycle_offset_next = axi_addr_next[OFFSET_WIDTH+2-1:2] + tr_dword_count_next;
                    last_cycle_next = output_cycle_count_next == 0;
                    input_active_next = 1'b1;

                    axi_addr_next = axi_addr_next + (tr_dword_count_next << 2);
                    op_dword_count_next = op_dword_count_next - tr_dword_count_next;
                    // number of bus transfers to AXI, DWORD count plus DWORD offset, divided by bus width in DWORDS
                    output_cycle_count_next = (tr_dword_count_next + axi_addr_next[OFFSET_WIDTH+2-1:2] - 1) >> (AXI_BURST_SIZE-2);
                    last_cycle_offset_next = axi_addr_next[OFFSET_WIDTH+2-1:2] + tr_dword_count_next;
                    last_cycle_next = output_cycle_count_next == 0;
                    input_active_next = 1'b1;

                    axi_addr_next = axi_addr_next + (tr_dword_count_next << 2);
                    op_dword_count_next = op_dword_count_next - tr_dword_count_next;

                    if (type_next == REQ_MEM_WRITE) begin
                        // write request
                        m_axi_awvalid_next = 1'b1;
                        if (AXIS_PCIE_DATA_WIDTH >= 256) begin
                            // some data is transferred with header
                            input_active_next = input_cycle_count_next > 0;
                            input_cycle_count_next = input_cycle_count_next - 1;
                            s_axis_cq_tready_next = 1'b0;
                            state_next = STATE_TRANSFER;
                        end else begin
                            s_axis_cq_tready_next = m_axi_wready_int_early;
                            state_next = STATE_TRANSFER;
                        end
                    end else begin
                        // invalid request
                        status_error_uncor_next = 1'b1;
                        if (s_axis_cq_tlast) begin
                            state_next = STATE_IDLE;
                        end else begin
                            s_axis_cq_tready_next = 1'b1;
                            state_next = STATE_WAIT_END;
                        end
                    end
                end else begin
                    state_next = STATE_IDLE;
                end
            end
        end

        STATE_HEADER: begin
            // header state, store rest of header (64 bit interface only)
            s_axis_cq_tready_next = m_axi_wready_int_early;

            if (s_axis_cq.tready && s_axis_cq.tvalid) begin
                transfer_in_save = 1'b1;

                // header fields
                op_dword_count_next = s_axis_cq.tdata[10:0];
                type_next = s_axis_cq.tdata[14:11];

                if (op_dword_count_next == 1) begin
                    // use first_be for both byte enables for single DWORD transfers
                    last_be_next = first_be_reg;
                end

                if (op_dword_count_next <= AXI_MAX_BURST_SIZE/4) begin
                    // packet smaller than max burst size (only for 64 bits)
                    // assumed to not cross 4k boundary, send one request
                    tr_dword_count_next = op_dword_count_next;
                end else begin
                    // packet larger than max burst size
                    // assumed to not cross 4k boundary, send one request
                    tr_dword_count_next = AXI_MAX_BURST_SIZE/4 - axi_addr_reg[OFFSET_WIDTH+2-1:2];
                end

                // required DWORD shift to place first DWORD from the TLP payload into proper position on AXI interface
                // bubble cycle required if first TLP payload transfer does not fill first AXI transfer
                offset_next = axi_addr_reg[OFFSET_WIDTH+2-1:2];
                bubble_cycle_next = 1'b0;
                first_cycle_offset_next = axi_addr_reg[OFFSET_WIDTH+2-1:2];
                first_cycle_next = 1'b1;

                // number of bus transfers in TLP, DOWRD count plus payload start DWORD offset, divided by bus width in DWORDS
                input_cycle_count_next = (tr_dword_count_next - 1) >> (AXI_BURST_SIZE-2);
                // number of bus transfers to AXI, DWORD count plus DWORD offset, divided by bus width in DWORDS
                output_cycle_count_next = (tr_dword_count_next + axi_addr_reg[OFFSET_WIDTH+2-1:2] - 1) >> (AXI_BURST_SIZE-2);
                last_cycle_offset_next = axi_addr_reg[OFFSET_WIDTH+2-1:2] + tr_dword_count_next;
                last_cycle_next = output_cycle_count_next == 0;
                input_active_next = 1'b1;

                m_axi_awaddr_next = axi_addr_reg;
                m_axi_awlen_next = output_cycle_count_next;

                axi_addr_next = axi_addr_reg + (tr_dword_count_next << 2);
                op_dword_count_next = op_dword_count_next - tr_dword_count_next;

                if (type_next == REQ_MEM_WRITE) begin
                    // write request
                    m_axi_awvalid_next = 1'b1;
                    s_axis_cq_tready_next = m_axi_wready_int_early;
                    state_next = STATE_TRANSFER;
                end else begin
                    // invalid request
                    status_error_uncor_next = 1'b1;
                    if (s_axis_cq.tlast) begin
                        state_next = STATE_IDLE;
                    end else begin
                        s_axis_cq_tready_next = 1'b1;
                        state_next = STATE_WAIT_END;
                    end
                end
            end else begin
                state_next = STATE_HEADER;
            end
        end

        STATE_TRANSFER: begin
            // transfer state, transfer data
            s_axis_cq_tready_next = m_axi_wready_int_early && input_active_reg && !(AXIS_PCIE_DATA_WIDTH >= 256 && first_cycle_reg && !bubble_cycle_reg);

            if (m_axi_wready_int_reg && ((s_axis_cq.tready && s_axis_cq.tvalid) || !input_active_reg || (AXIS_PCIE_DATA_WIDTH >= 256 && first_cycle_reg && !bubble_cycle_reg))) begin
                transfer_in_save = s_axis_cq.tready && s_axis_cq.tvalid;

                // transfer data
                if (AXIS_PCIE_DATA_WIDTH >= 256 && first_cycle_reg && !bubble_cycle_reg) begin
                    m_axi_wdata_int = {save_axis_tdata_reg, {AXIS_PCIE_DATA_WIDTH{1'b0}}} >> ((AXI_STRB_WIDTH/4-offset_reg)*32);
                    s_axis_cq_tready_next = m_axi_wready_int_early && input_active_reg;
                end else begin
                    m_axi_wdata_int = shift_axis_tdata;
                end
                // generate strb signal
                if (first_cycle_reg) begin
                    m_axi_wstrb_int = {{AXI_STRB_WIDTH-4{1'b1}}, first_be_reg} << (first_cycle_offset_reg*4);
                end else begin
                    m_axi_wstrb_int = {AXI_STRB_WIDTH{1'b1}};
                end

                // update cycle counters
                if (input_active_reg && !(AXIS_PCIE_DATA_WIDTH >= 256 && first_cycle_reg && !bubble_cycle_reg)) begin
                    input_cycle_count_next = input_cycle_count_reg - 1;
                    input_active_next = input_cycle_count_reg > 0;
                end
                output_cycle_count_next = output_cycle_count_reg - 1;
                last_cycle_next = output_cycle_count_next == 0;

                // modify strb signal at end of transfer
                if (last_cycle_reg) begin
                    if (op_dword_count_reg == 0) begin
                        if (last_cycle_offset_reg > 0) begin
                            m_axi_wstrb_int = m_axi_wstrb_int & {last_be_reg, {AXI_STRB_WIDTH-4{1'b1}}} >> (AXI_STRB_WIDTH-last_cycle_offset_reg*4);
                        end else begin
                            m_axi_wstrb_int = m_axi_wstrb_int & {last_be_reg, {AXI_STRB_WIDTH-4{1'b1}}};
                        end
                    end
                    m_axi_wlast_int = 1'b1;
                end
                m_axi_wvalid_int = 1'b1;
                first_cycle_next = 1'b0;
                if (!last_cycle_reg) begin
                    s_axis_cq_tready_next = m_axi_wready_int_early && input_active_next;
                    state_next = STATE_TRANSFER;
                end else if (op_dword_count_reg > 0) begin
                    // current transfer done, but operation not finished yet
                    if (op_dword_count_reg <= AXI_MAX_BURST_SIZE/4) begin
                        // packet smaller than max burst size
                        // assumed to not cross 4k boundary, send one request
                        tr_dword_count_next = op_dword_count_reg;
                        m_axi_awlen_next = (tr_dword_count_next + axi_addr_reg[OFFSET_WIDTH+2-1:2] - 1) >> (AXI_BURST_SIZE-2);
                    end else begin
                        // packet larger than max burst size
                        // assumed to not cross 4k boundary, send one request
                        tr_dword_count_next = AXI_MAX_BURST_SIZE/4 - axi_addr_reg[OFFSET_WIDTH+2-1:2];
                        m_axi_awlen_next = (tr_dword_count_next - 1) >> (AXI_BURST_SIZE-2);
                    end

                    m_axi_awaddr_next = axi_addr_reg;

                    // keep offset, no bubble cycles, not first cycle
                    bubble_cycle_next = 1'b0;
                    first_cycle_next = 1'b0;

                    // number of bus transfers in TLP, DOWRD count minus payload start DWORD offset, divided by bus width in DWORDS
                    input_cycle_count_next = (tr_dword_count_next - offset_reg - 1) >> (AXI_BURST_SIZE-2);
                    // number of bus transfers to AXI, DWORD count plus DWORD offset, divided by bus width in DWORDS
                    output_cycle_count_next = (tr_dword_count_next + axi_addr_reg[OFFSET_WIDTH+2-1:2] - 1) >> (AXI_BURST_SIZE-2);
                    last_cycle_offset_next = axi_addr_reg[OFFSET_WIDTH+2-1:2] + tr_dword_count_next;
                    last_cycle_next = output_cycle_count_next == 0;
                    input_active_next = tr_dword_count_next > offset_reg;

                    axi_addr_next = axi_addr_reg + (tr_dword_count_next << 2);
                    op_dword_count_next = op_dword_count_reg - tr_dword_count_next;

                    m_axi_awvalid_next = 1'b1;
                    s_axis_cq_tready_next = m_axi_wready_int_early && input_active_next;
                    state_next = STATE_TRANSFER;
                end else begin
                    s_axis_cq_tready_next = m_axi_wready_int_early && (!m_axi_awvalid || m_axi_awready);
                    state_next = STATE_IDLE;
                end
            end else begin
                state_next = STATE_TRANSFER;
            end
        end

        STATE_WAIT_END: begin
            // wait end state, wait for end of TLP
            s_axis_cq_tready_next = 1'b1;

            if (s_axis_cq.tready & s_axis_cq.tvalid) begin
                if (s_axis_cq.tlast) begin
                    if (AXIS_PCIE_DATA_WIDTH > 64) begin
                        s_axis_cq_tready_next = m_axi_wready_int_early && (!m_axi_awvalid || m_axi_awready);
                    end else begin
                        s_axis_cq_tready_next = 1'b1;
                    end
                    state_next = STATE_IDLE;
                end else begin
                    state_next = STATE_WAIT_END;
                end
            end else begin
                state_next = STATE_WAIT_END;
            end
        end

        default: state_next = STATE_IDLE;

    endcase
end

always_ff @(posedge clk) begin
    if (rst) begin
        state_reg <= STATE_IDLE;
        s_axis_cq_tready_reg <= 1'b0;

        m_axi_awvalid_reg <= 1'b0;

        status_error_uncor_reg <= 1'b0;
    end else begin
        state_reg <= state_next;
        s_axis_cq_tready_reg <= s_axis_cq_tready_next;

        m_axi_awvalid_reg <= m_axi_awvalid_next;

        status_error_uncor_reg <= status_error_uncor_next;
    end

    axi_addr_reg <= axi_addr_next;
    op_dword_count_reg <= op_dword_count_next;
    tr_dword_count_reg <= tr_dword_count_next;
    input_cycle_count_reg <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg <= input_active_next;
    bubble_cycle_reg <= bubble_cycle_next;
    first_cycle_reg <= first_cycle_next;
    last_cycle_reg <= last_cycle_next;

    type_reg <= type_next;
    first_be_reg <= first_be_next;
    last_be_reg <= last_be_next;
    offset_reg <= offset_next;
    first_cycle_offset_reg <= first_cycle_offset_next;
    last_cycle_offset_reg <= last_cycle_offset_next;

    m_axi_awaddr_reg <= m_axi_awaddr_next;
    m_axi_awlen_reg <= m_axi_awlen_next;

    if (transfer_in_save) begin
        save_axis_tdata_reg <= s_axis_cq.tdata;
    end
end

// output datapath logic (AXI write data)
logic [AXI_DATA_WIDTH-1:0] m_axi_wdata_reg = {AXI_DATA_WIDTH{1'b0}};
logic [AXI_STRB_WIDTH-1:0] m_axi_wstrb_reg = {AXI_STRB_WIDTH{1'b0}};
logic                      m_axi_wvalid_reg = 1'b0, m_axi_wvalid_next;
logic                      m_axi_wlast_reg = 1'b0;

logic [AXI_DATA_WIDTH-1:0] temp_m_axi_wdata_reg = {AXI_DATA_WIDTH{1'b0}};
logic [AXI_STRB_WIDTH-1:0] temp_m_axi_wstrb_reg = {AXI_STRB_WIDTH{1'b0}};
logic                      temp_m_axi_wvalid_reg = 1'b0, temp_m_axi_wvalid_next;
logic                      temp_m_axi_wlast_reg = 1'b0;

// datapath control
logic store_axi_w_int_to_output;
logic store_axi_w_int_to_temp;
logic store_axi_w_temp_to_output;

assign m_axi_wdata = m_axi_wdata_reg;
assign m_axi_wstrb = m_axi_wstrb_reg;
assign m_axi_wvalid = m_axi_wvalid_reg;
assign m_axi_wlast = m_axi_wlast_reg;

// enable ready input next cycle if output is ready or if both output registers are empty
assign m_axi_wready_int_early = m_axi_wready || (!temp_m_axi_wvalid_reg && !m_axi_wvalid_reg);

always_comb begin
    m_axi_wvalid_reg <= m_axi_wvalid_next;
    m_axi_wready_int_reg <= m_axi_wready_int_early;
    temp_m_axi_wvalid_reg <= temp_m_axi_wvalid_next;

    // datapath
    if (store_axi_w_int_to_output) begin
        m_axi_wdata_reg <= m_axi_wdata_int;
        m_axi_wstrb_reg <= m_axi_wstrb_int;
        m_axi_wlast_reg <= m_axi_wlast_int;
    end else if (store_axi_w_temp_to_output) begin
        m_axi_wdata_reg <= temp_m_axi_wdata_reg;
        m_axi_wstrb_reg <= temp_m_axi_wstrb_reg;
        m_axi_wlast_reg <= temp_m_axi_wlast_reg;
    end

    if (store_axi_w_int_to_temp) begin
        temp_m_axi_wdata_reg <= m_axi_wdata_int;
        temp_m_axi_wstrb_reg <= m_axi_wstrb_int;
        temp_m_axi_wlast_reg <= m_axi_wlast_int;
    end

    if (rst) begin
        m_axi_wvalid_reg <= 1'b0;
        m_axi_wready_int_reg <= 1'b0;
        temp_m_axi_wvalid_reg <= 1'b0;
    end
end


endmodule

`resetall
