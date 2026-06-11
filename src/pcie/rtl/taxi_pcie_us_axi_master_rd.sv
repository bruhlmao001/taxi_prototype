`resetall
`timescale 1ns / 1ps
`default_nettype none

module pcie_us_axi_master_rd #(
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
     * TLP output (completion)
     */
    taxi_axis_if.src      m_axis_cc,

     /*
     * AXI Master Read output
     */
    taxi_axi_if.rd_mst     m_axi_rd, 

   
     /*
     * Configuration
     */
    input  wire [15:0]                        completer_id,
    input  wire                               completer_id_enable,
    input  wire [2:0]                         max_payload_size,

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


localparam AXIS_PCIE_CC_USER_W = m_axis_cc.USER_W;
localparam AXI_DATA_W = m_axi_rd.DATA_W;
localparam AXI_ADDR_W = m_axi_rd.ADDR_W;
localparam AXI_STRB_W = m_axi_rd.STRB_W;
localparam AXI_MAX_BURST_LEN = m_axi_rd.MAX_BURST_LEN; // maximum burst length in dwords 

localparam AXI_WORD_WIDTH = AXI_STRB_W; // number of bytes in an AXI word
localparam  AXI_WORD_SIZE = AXI_DATA_W/AXI_WORD_WIDTH;
localparam AXI_BURST_SIZE = $clog2(AXI_STRB_W);
localparam AXI_MAX_BURST_SIZE = AXI_MAX_BURST_LEN*AXI_WORD_WIDTH;

parameter PAYLOAD_MAX = AXI_MAX_BURST_SIZE < 4096 ? $clog2(AXI_MAX_BURST_SIZE/128) : 5;

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
    AXI_STATE_IDLE = 2'b00,
    AXI_STATE_HEADER = 2'b01,
    AXI_STATE_START = 2'b10,
    AXI_STATE_WAIT_END = 2'b11
} axi_state_t;

logic [1:0] axi_state_reg = AXI_STATE_IDLE, axi_state_next;

tlp_state_t tlp_state_reg = TLP_STATE_IDLE, tlp_state_next;

typedef enum logic [2:0] {
    TLP_STATE_IDLE = 3'd0,
    TLP_STATE_HEADER_1 = 3'd1,
    TLP_STATE_HEADER_2 = 3'd2,
    TLP_STATE_TRANSFER = 3'd3,
    TLP_STATE_CPL_1 = 3'd4,
    TLP_STATE_CPL_2 = 3'd5;
} tlp_state_t;

logic [2:0] tlp_state_reg = TLP_STATE_IDLE, tlp_state_next;

// datapath control signals
logic transfer_in_save;
logic tlp_command_ready;

reg [1:0] first_be_offset;
reg [1:0] last_be_offset;
reg [2:0] single_dword_len;


logic [AXI_ADDR_W-1:0] axi_addr_reg ='0 , axi_addr_next;
logic [10:0] op_dword_count_reg = '0, op_dword_count_next;
logic [10:0] tr_dword_count_reg = '0, tr_dword_count_next;
logic [12:0] input_cycle_count_reg = '0, input_cycle_count_next;
logic [12:0] output_cycle_count_reg = '0, output_cycle_count_next;
logic input_active_reg = 1'b0, input_active_next;
logic first_cycle_reg = 1'b0, first_cycle_next;
logic last_cycle_reg = 1'b0, last_cycle_next;
logic bubble_cycle_reg = 1'b0, bubble_cycle_next;

logic last_tlp_reg = 1'b0, last_tlp_next;
logic [2:0] status_reg = 3'd0, status_next;
logic [15:0] requester_id_reg = 16'd0, requester_id_next;
logic [7:0] tag_reg = 8'd0, tag_next;
logic [2:0] tc_reg = 3'd0, tc_next;
logic [2:0] attr_reg = 3'd0, attr_next;

logic [6:0] tlp_cmd_lower_addr_reg = 7'd0, tlp_cmd_lower_addr_next;
logic [12:0] tlp_cmd_byte_len_reg = 13'd0, tlp_cmd_byte_len_next;
logic [10:0] tlp_cmd_dword_len_reg = 11'd0, tlp_cmd_dword_len_next;
logic [9:0] tlp_cmd_input_cycle_len_reg = 10'd0, tlp_cmd_input_cycle_len_next;
logic [9:0] tlp_cmd_output_cycle_len_reg = 10'd0, tlp_cmd_output_cycle_len_next;
logic [OFFSET_WIDTH-1:0] tlp_cmd_offset_reg = {OFFSET_WIDTH{1'b0}}, tlp_cmd_offset_next;
logic [2:0] tlp_cmd_status_reg = 3'd0, tlp_cmd_status_next;
logic [15:0] tlp_cmd_requester_id_reg = 16'd0, tlp_cmd_requester_id_next;
logic [7:0] tlp_cmd_tag_reg = 8'd0, tlp_cmd_tag_next;
logic [2:0] tlp_cmd_tc_reg = 3'd0, tlp_cmd_tc_next;
logic [2:0] tlp_cmd_attr_reg = 3'd0, tlp_cmd_attr_next;
logic tlp_cmd_bubble_cycle_reg = 1'b0, tlp_cmd_bubble_cycle_next;
logic tlp_cmd_last_reg = 1'b0, tlp_cmd_last_next;
logic tlp_cmd_valid_reg = 1'b0, tlp_cmd_valid_next;

logic [10:0] max_payload_size_dw_reg = 11'd0;

logic s_axis_cq_tready_reg = 1'b0, s_axis_cq_tready_next;

logic [AXI_ADDR_WIDTH-1:0] m_axi_araddr_reg = {AXI_ADDR_WIDTH{1'b0}}, m_axi_araddr_next;
logic [7:0] m_axi_arlen_reg = 8'd0, m_axi_arlen_next;
logic m_axi_arvalid_reg = 1'b0, m_axi_arvalid_next;
logic m_axi_rready_reg = 1'b0, m_axi_rready_next;

logic [AXI_DATA_WIDTH-1:0] save_axi_rdata_reg = {AXI_DATA_WIDTH{1'b0}};

wire [AXI_DATA_WIDTH-1:0] shift_axi_rdata = {m_axi_rdata, save_axi_rdata_reg} >> ((AXI_STRB_WIDTH/4-offset_reg)*32);

logic status_err_cor_reg = 1'b0, status_err_cor_next;
logic status_err_uncor_reg = 1'b0, status_err_uncor_next;

// internal datapath
logic  [AXIS_PCIE_DATA_WIDTH-1:0]    m_axis_cc_tdata_int;
logic  [AXIS_PCIE_KEEP_WIDTH-1:0]    m_axis_cc_tkeep_int;
logic                                m_axis_cc_tvalid_int;
logic                                m_axis_cc_tready_int_reg = 1'b0;
logic                                m_axis_cc_tlast_int;
logic  [AXIS_PCIE_CC_USER_WIDTH-1:0] m_axis_cc_tuser_int;
logic                                m_axis_cc_tready_int_early;


logic [3:0] type_reg = 4'd0, type_next;
logic [3:0] first_be_reg = 4'd0, first_be_next;
logic [3:0] last_be_reg = 4'd0, last_be_next;
logic [OFFSET_WIDTH-1:0] offset_reg = {OFFSET_WIDTH{1'b0}}, offset_next;
logic [OFFSET_WIDTH-1:0] last_cycle_offset_reg = {OFFSET_WIDTH{1'b0}}, last_cycle_offset_next;

logic [CHUNK_W-1:0] resp_chunk_reg = 0, resp_chunk_next;

assign s_axis_cq.tready = s_axis_cq_tready_reg;

assign m_axi_rd.arid = '0;
assign m_axi_rd.araddr = m_axi_araddr_reg;
assign m_axi_rd.arlen = m_axi_arlen_reg;
assign m_axi_rd.arsize = AXI_BURST_SIZE;
assign m_axi_rd.arburst = 2'b01;
assign m_axi_rd.arlock = 1'b0;
assign m_axi_rd.arcache = 4'b0011;
assign m_axi_rd.arprot = 3'b010;
assign m_axi_rd.awqos = '0;
assign m_axi_rd.awregion = '0;
assign m_axi_rd.arvalid = m_axi_arvalid_reg;
assign m_axi_rd.rready = m_axi_rready_reg;

assign status_err_cor = status_err_cor_reg;
assign status_err_uncor = status_err_uncor_reg;

always_comb begin
    casez (first_be_next)
        4'b0000: single_dword_len = 3'd1;
        4'b0001: single_dword_len = 3'd1;
        4'b0010: single_dword_len = 3'd1;
        4'b0100: single_dword_len = 3'd1;
        4'b1000: single_dword_len = 3'd1;
        4'b0011: single_dword_len = 3'd2;
        4'b0110: single_dword_len = 3'd2;
        4'b1100: single_dword_len = 3'd2;
        4'b01z1: single_dword_len = 3'd3;
        4'b1z10: single_dword_len = 3'd3;
        4'b1zz1: single_dword_len = 3'd4;
    endcase

    casez (first_be_next)
        4'b0000: first_be_offset = 2'b00;
        4'bzzz1: first_be_offset = 2'b00;
        4'bzz10: first_be_offset = 2'b01;
        4'bz100: first_be_offset = 2'b10;
        4'b1000: first_be_offset = 2'b11;
    endcase

    casez (last_be_next)
        4'b0000: last_be_offset = 2'b00;
        4'b1zzz: last_be_offset = 2'b00;
        4'b01zz: last_be_offset = 2'b01;
        4'b001z: last_be_offset = 2'b10;
        4'b0001: last_be_offset = 2'b11;
    endcase
end

always_comb begin
    axi_state_next = AXI_STATE_IDLE;

    s_axis_cq_tready_next = 1'b0;

    m_axi_araddr_next = m_axi_araddr_reg;
    m_axi_arlen_next = m_axi_arlen_reg;
    m_axi_arvalid_next = m_axi_arvalid_reg && !m_axi_arready;

    axi_addr_next = axi_addr_reg;
    op_count_next = op_count_reg;
    op_dword_count_next = op_dword_count_reg;
    tlp_dword_count_next = tlp_dword_count_reg;
    first_be_next = first_be_reg;
    last_be_next = last_be_reg;

    tlp_cmd_lower_addr_next = tlp_cmd_lower_addr_reg;
    tlp_cmd_byte_len_next = tlp_cmd_byte_len_reg;
    tlp_cmd_dword_len_next = tlp_cmd_dword_len_reg;
    tlp_cmd_input_cycle_len_next = tlp_cmd_input_cycle_len_reg;
    tlp_cmd_output_cycle_len_next = tlp_cmd_output_cycle_len_reg;
    tlp_cmd_offset_next = tlp_cmd_offset_reg;
    tlp_cmd_status_next = tlp_cmd_status_reg;
    tlp_cmd_requester_id_next = tlp_cmd_requester_id_reg;
    tlp_cmd_tag_next = tlp_cmd_tag_reg;
    tlp_cmd_attr_next = tlp_cmd_attr_reg;
    tlp_cmd_tc_next = tlp_cmd_tc_reg;
    tlp_cmd_bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
    tlp_cmd_last_next = tlp_cmd_last_reg;
    tlp_cmd_valid_next = tlp_cmd_valid_reg && !tlp_cmd_ready;

    status_error_cor_next = 1'b0;
    status_error_uncor_next = 1'b0;

    // TLP segmentation and AXI read request generation
    case (axi_state_reg)
        AXI_STATE_IDLE: 
        begin
            // idle state, wait for completion request
            s_axis_cq_tready_next = !tlp_cmd_valid_reg;

            if (s_axis_cq.tready & s_axis_cq.tvalid) begin
                // header fields
                axi_addr_next = {s_axis_cq.tdata[63:2], first_be_offset};
                tlp_cmd_status_next = CPL_STATUS_SC; // successful completion
                if (AXIS_PCIE_DATA_WIDTH > 64) begin
                    op_dword_count_next = s_axis_cq.tdata[74:64];
                    if (op_dword_count_next == 1) begin
                        op_count_next = single_dword_len;
                    end else begin
                        op_count_next = (op_dword_count_next << 2) - first_be_offset - last_be_offset;
                    end
                    tlp_cmd_requester_id_next = s_axis_cq.tdata[95:80];
                    tlp_cmd_tag_next = s_axis_cq.tdata[103:96];
                    tlp_cmd_tc_next = s_axis_cq.tdata[123:121];
                    tlp_cmd_attr_next = s_axis_cq.tdata[126:124];
                end

                // tuser fields
                if (AXIS_PCIE_DATA_WIDTH == 512) begin
                    first_be_next = s_axis_cq.tuser[3:0];
                    last_be_next = s_axis_cq.tuser[11:8];
                end else begin
                    first_be_next = s_axis_cq.tuser[3:0];
                    last_be_next = s_axis_cq.tuser[7:4];
                end

                if (AXIS_PCIE_DATA_WIDTH == 64) begin
                    // 64 bit interface hasn't processed the whole header yet
                    s_axis_cq_tready_next = 1'b1;
                    if (s_axis_cq.tlast) begin
                        // truncated packet
                        // report uncorrectable error
                        status_error_uncor_next = 1'b1;
                        axi_state_next = AXI_STATE_IDLE;
                    end else begin
                        axi_state_next = AXI_STATE_HEADER;
                    end
                end else begin
                    // processed whole header; check request type
                    if (s_axis_cq.tdata[78:75] == REQ_MEM_READ) begin
                        // read request
                        s_axis_cq_tready_next = 1'b0;
                        axi_state_next = AXI_STATE_START;
                    end else if (s_axis_cq.tdata[78:75] == REQ_MEM_WRITE || (s_axis_cq.tdata[78:75] & 4'b1100) == REQ_MSG) begin
                        // posted request (memory write or message), drop and report uncorrectable error
                        status_error_uncor_next = 1'b1;
                        if (s_axis_cq.tlast) begin
                            axi_state_next = AXI_STATE_IDLE;
                        end else begin
                            s_axis_cq_tready_next = 1'b1;
                            axi_state_next = AXI_STATE_WAIT_END;
                        end
                    end else begin
                        // invalid request, send UR completion
                        tlp_cmd_status_next = CPL_STATUS_UR; // unsupported request
                        tlp_cmd_valid_next = 1'b1;
                        // report correctable error
                        status_error_cor_next = 1'b1;
                        if (s_axis_cq.tlast) begin
                            axi_state_next = AXI_STATE_IDLE;
                        end else begin
                            s_axis_cq_tready_next = 1'b1;
                            axi_state_next = AXI_STATE_WAIT_END;
                        end
                    end
                end
            end else begin
                axi_state_next = AXI_STATE_IDLE;
            end
        end 

        AXI_STATE_HEADER: begin
            // header state, store rest of header (64 bit interface only)
            s_axis_cq_tready_next = 1'b1;

            if (s_axis_cq.tready & s_axis_cq.tvalid) begin
                // header fields
                op_dword_count_next = s_axis_cq.tdata[10:0];
                if (op_dword_count_next == 1) begin
                    op_count_next = single_dword_len;
                end else begin
                    op_count_next = (op_dword_count_next << 2) - first_be_offset - last_be_offset;
                end
                tlp_cmd_requester_id_next = s_axis_cq.tdata[31:16];
                tlp_cmd_tag_next = s_axis_cq.tdata[39:32];
                tlp_cmd_tc_next = s_axis_cq.tdata[59:57];
                tlp_cmd_attr_next = s_axis_cq.tdata[62:60];

                // processed whole header; check request type
                if (s_axis_cq.tdata[14:11] == REQ_MEM_READ) begin
                    // read request
                    s_axis_cq_tready_next = 1'b0;
                    axi_state_next = AXI_STATE_START;
                end else if (s_axis_cq.tdata[14:11] == REQ_MEM_WRITE || (s_axis_cq.tdata[14:11] & 4'b1100) == REQ_MSG)
                begin
                    // posted request (memory write or message), drop and report uncorrectable error
                    // write request - drop
                    status_error_uncor_next = 1'b1;
                    if (s_axis_cq.tlast) begin
                        axi_state_next = AXI_STATE_IDLE;
                    end else begin
                        s_axis_cq_tready_next = 1'b1;
                        axi_state_next = AXI_STATE_WAIT_END;
                    end
                end else begin
                    // invalid request, send UR completion
                    tlp_cmd_status_next = CPL_STATUS_UR; // unsupported request
                    tlp_cmd_valid_next = 1'b1;
                    // report correctable error
                    status_error_cor_next = 1'b1;
                    if (s_axis_cq.tlast) begin
                        axi_state_next = AXI_STATE_IDLE;
                    end else begin
                        s_axis_cq_tready_next = 1'b1;
                        axi_state_next = AXI_STATE_WAIT_END;
                    end
                end
            end else begin
                axi_state_next = AXI_STATE_HEADER;
            end
        end

         AXI_STATE_WAIT_END: begin
            // wait end state, wait for end of TLP
            s_axis_cq_tready_next = 1'b1;

            if (s_axis_cq.tready & s_axis_cq.tvalid) begin
                if (s_axis_cq.tlast) begin
                    s_axis_cq_tready_next = !tlp_cmd_valid_reg;
                    axi_state_next = AXI_STATE_IDLE;
                end else begin
                    axi_state_next = AXI_STATE_WAIT_END;
                end
            end else begin
                axi_state_next = AXI_STATE_WAIT_END;
            end
        end

         default: axi_state_next = AXI_STATE_IDLE;
      
    endcase

always_comb begin
    tlp_state_next = TLP_STATE_IDLE;

    transfer_in_save = 1'b0;

    tlp_cmd_ready = 1'b0;

    m_axi_rready_next = 1'b0;

    tlp_lower_addr_next = tlp_lower_addr_reg;
    tlp_len_next = tlp_len_reg;
    dword_count_next = dword_count_reg;
    offset_next = offset_reg;
    input_cycle_count_next = input_cycle_count_reg;
    output_cycle_count_next = output_cycle_count_reg;
    input_active_next = input_active_reg;
    bubble_cycle_next = bubble_cycle_reg;
    last_cycle_next = last_cycle_reg;
    last_tlp_next = last_tlp_reg;
    status_next = status_reg;
    requester_id_next = requester_id_reg;
    tag_next = tag_reg;
    tc_next = tc_reg;
    attr_next = attr_reg;

    m_axis_cc_tdata_int = {AXIS_PCIE_DATA_WIDTH{1'b0}};
    m_axis_cc_tkeep_int = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
    m_axis_cc_tvalid_int = 1'b0;
    m_axis_cc_tlast_int = 1'b0;
    m_axis_cc_tuser_int = {AXIS_PCIE_CC_USER_WIDTH{1'b0}};

    m_axis_cc_tdata_int[6:0] = tlp_lower_addr_reg; // lower address
    m_axis_cc_tdata_int[9:8] = 2'b00; // AT
    m_axis_cc_tdata_int[28:16] = tlp_len_reg; // byte count
    m_axis_cc_tdata_int[42:32] = dword_count_reg;
    m_axis_cc_tdata_int[45:43] = status_reg;
    m_axis_cc_tdata_int[63:48] = requester_id_reg;
    if (AXIS_PCIE_DATA_WIDTH > 64) begin
        m_axis_cc_tdata_int[71:64] = tag_reg;
        m_axis_cc_tdata_int[87:72] = completer_id;
        m_axis_cc_tdata_int[88] = completer_id_enable;
        m_axis_cc_tdata_int[91:89] = tc_reg;
        m_axis_cc_tdata_int[94:92] = attr_reg;
        m_axis_cc_tdata_int[95] = 1'b0; // force ECRC
        if (AXIS_PCIE_DATA_WIDTH == 256) begin
            m_axis_cc_tdata_int[255:96] = shift_axi_rdata[255:96];
        end else begin
            m_axis_cc_tdata_int[127:96] = shift_axi_rdata[127:96];
        end
    end

    if (AXIS_PCIE_DATA_WIDTH == 512) begin
        m_axis_cc_tkeep_int = 16'b0000000000000111;
    end else if (AXIS_PCIE_DATA_WIDTH == 256) begin
        m_axis_cc_tkeep_int = 8'b00000111;
    end else if (AXIS_PCIE_DATA_WIDTH == 128) begin
        m_axis_cc_tkeep_int = 4'b0111;
    end else if (AXIS_PCIE_DATA_WIDTH == 64) begin
        m_axis_cc_tkeep_int = 2'b11;
    end

    if (AXIS_PCIE_DATA_WIDTH == 512) begin
        m_axis_cc_tuser_int[1:0] = 2'b01; // is_sop
        m_axis_cc_tuser_int[3:2] = 2'd0; // is_sop0_ptr
        m_axis_cc_tuser_int[5:4] = 2'd0; // is_sop1_ptr
        m_axis_cc_tuser_int[7:6] = 2'b01; // is_eop
        m_axis_cc_tuser_int[11:8]  = 4'd3; // is_eop0_ptr
        m_axis_cc_tuser_int[15:12] = 4'd0; // is_eop1_ptr
        m_axis_cc_tuser_int[16] = 1'b0; // discontinue
        m_axis_cc_tuser_int[80:17] = 64'd0; // parity
    end else begin
        m_axis_cc_tuser_int[0] = 1'b0; // discontinue
        m_axis_cc_tuser_int[32:1] = 32'd0; // parity
    end


    // AXI read response processing and TLP generation
    case (tlp_state_reg)
        TLP_STATE_IDLE: begin
            // idle state, wait for command
            m_axi_rready_next = 1'b0;

            // store TLP fields and transfer parameters
            tlp_lower_addr_next = tlp_cmd_lower_addr_reg;
            tlp_len_next = tlp_cmd_byte_len_reg;
            dword_count_next = tlp_cmd_dword_len_reg;
            offset_next = tlp_cmd_offset_reg;
            input_cycle_count_next = tlp_cmd_input_cycle_len_reg;
            output_cycle_count_next = tlp_cmd_output_cycle_len_reg;
            input_active_next = 1'b1;
            bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
            last_cycle_next = tlp_cmd_output_cycle_len_reg == 0;
            last_tlp_next = tlp_cmd_last_reg;
            status_next = tlp_cmd_status_reg;
            requester_id_next = tlp_cmd_requester_id_reg;
            tag_next = tlp_cmd_tag_reg;
            tc_next = tlp_cmd_tc_reg;
            attr_next = tlp_cmd_attr_reg;

            if (tlp_cmd_valid_reg) begin
                tlp_cmd_ready = 1'b1;
                if (status_next == CPL_STATUS_SC) begin
                    // SC status, output TLP header
                    if (AXIS_PCIE_DATA_WIDTH == 64) begin
                        m_axi_rready_next = 1'b0;
                    end else begin
                        m_axi_rready_next = m_axis_cc_tready_int_early;
                    end
                    tlp_state_next = TLP_STATE_HEADER_1;
                end else begin
                    // status other than SC
                    tlp_state_next = TLP_STATE_CPL_1;
                end
            end else begin
                tlp_state_next = TLP_STATE_IDLE;
            end
        end

        TLP_STATE_HEADER_1: begin
            // header 1 state, send TLP header
            if (AXIS_PCIE_DATA_WIDTH == 64) begin
                m_axi_rready_next = 1'b0;

                if (m_axis_cc_tready_int_reg) begin
                    // output first part of header
                    m_axis_cc_tvalid_int = 1'b1;

                    m_axi_rready_next = m_axis_cc_tready_int_early;

                    tlp_state_next = TLP_STATE_HEADER_2;
                end else begin
                    tlp_state_next = TLP_STATE_HEADER_1;
                end
            end else begin
                m_axi_rready_next = m_axis_cc_tready_int_early && input_active_reg;

                if (m_axis_cc_tready_int_reg && ((m_axi_rd.rready && m_axi_rd.rvalid) || !input_active_reg)) begin
                    transfer_in_save = m_axi_rd.rready && m_axi_rd.rvalid;

                    if (AXIS_PCIE_DATA_WIDTH >= 256 && bubble_cycle_reg) begin
                        // bubble cycle; store input data and update input cycle count
                        if (input_active_reg) begin
                            input_cycle_count_next = input_cycle_count_reg - 1;
                            input_active_next = input_cycle_count_reg > 0;
                        end
                        bubble_cycle_next = 1'b0;
                        m_axi_rready_next = m_axis_cc_tready_int_early && input_active_next;
                        tlp_state_next = TLP_STATE_HEADER_1;
                    end else begin
                        // some data is transferred with header
                        dword_count_next = dword_count_reg - (AXIS_PCIE_KEEP_WIDTH-3);
                        // update cycle counters
                        if (input_active_reg) begin
                            input_cycle_count_next = input_cycle_count_reg - 1;
                            input_active_next = input_cycle_count_reg > 0;
                        end
                        output_cycle_count_next = output_cycle_count_reg - 1;
                        last_cycle_next = output_cycle_count_next == 0;

                        // transfer data
                        m_axis_cc_tdata_int[AXIS_PCIE_DATA_WIDTH-1:96] = shift_axi_rdata[AXIS_PCIE_DATA_WIDTH-1:96];

                        // generate tvalid and tkeep signals for header and data
                        m_axis_cc_tvalid_int = 1'b1;
                        if (dword_count_reg >= AXIS_PCIE_KEEP_WIDTH-3) begin
                            m_axis_cc_tkeep_int = {AXIS_PCIE_KEEP_WIDTH{1'b1}};
                        end else begin
                            m_axis_cc_tkeep_int = {AXIS_PCIE_KEEP_WIDTH{1'b1}} >> (AXIS_PCIE_KEEP_WIDTH-3 - dword_count_reg);
                        end

                        if (last_cycle_reg) begin
                            m_axis_cc_tlast_int = 1'b1;

                            // skip idle state if possible
                            tlp_lower_addr_next = tlp_cmd_lower_addr_reg;
                            tlp_len_next = tlp_cmd_byte_len_reg;
                            dword_count_next = tlp_cmd_dword_len_reg;
                            offset_next = tlp_cmd_offset_reg;
                            input_cycle_count_next = tlp_cmd_input_cycle_len_reg;
                            output_cycle_count_next = tlp_cmd_output_cycle_len_reg;
                            input_active_next = 1'b1;
                            bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
                            last_cycle_next = tlp_cmd_output_cycle_len_reg == 0;
                            last_tlp_next = tlp_cmd_last_reg;
                            status_next = tlp_cmd_status_reg;
                            requester_id_next = tlp_cmd_requester_id_reg;
                            tag_next = tlp_cmd_tag_reg;
                            tc_next = tlp_cmd_tc_reg;
                            attr_next = tlp_cmd_attr_reg;

                            if (tlp_cmd_valid_reg) begin
                                tlp_cmd_ready = 1'b1;
                                m_axi_rready_next = m_axis_cc_tready_int_early;
                                tlp_state_next = TLP_STATE_HEADER_1;
                            end else begin
                                m_axi_rready_next = 1'b0;
                                tlp_state_next = TLP_STATE_IDLE;
                            end
                        end else begin
                            m_axi_rready_next = m_axis_cc_tready_int_early && input_active_next;
                            tlp_state_next = TLP_STATE_TRANSFER;
                        end
                    end
                end else begin
                    tlp_state_next = TLP_STATE_HEADER_1;
                end
            end
        end

        TLP_STATE_HEADER_2: begin
            // header 2 state, send rest of TLP header (64 bit interface only)
            m_axi_rready_next = m_axis_cc_tready_int_early && input_active_reg;

            m_axis_cc_tdata_int[7:0] = tag_reg;
            m_axis_cc_tdata_int[23:8] = completer_id;
            m_axis_cc_tdata_int[24] = completer_id_enable;
            m_axis_cc_tdata_int[27:25] = tc_reg;
            m_axis_cc_tdata_int[30:28] = attr_reg;
            m_axis_cc_tdata_int[31] = 1'b0; // force ECRC
            m_axis_cc_tdata_int[63:32] = shift_axi_rdata[63:32];

            if (m_axis_cc_tready_int_reg && ((m_axi_rready && m_axi_rvalid) || !input_active_reg)) begin
                transfer_in_save = m_axi_rready && m_axi_rvalid;

                // some data is transferred with header
                dword_count_next = dword_count_reg - 1;
                // update cycle counters
                if (input_active_reg) begin
                    input_cycle_count_next = input_cycle_count_reg - 1;
                    input_active_next = input_cycle_count_reg > 0;
                end
                output_cycle_count_next = output_cycle_count_reg - 1;
                last_cycle_next = output_cycle_count_next == 0;

                // generate tvalid and tkeep signals for header and data
                m_axis_cc_tvalid_int = 1'b1;
                if (dword_count_reg >= 1) begin
                    m_axis_cc_tkeep_int = 2'b11;
                end else begin
                    m_axis_cc_tkeep_int = 2'b11 >> (1 - dword_count_reg);
                end

                if (last_cycle_reg) begin
                    m_axis_cc_tlast_int = 1'b1;

                    // skip idle state if possible
                    tlp_lower_addr_next = tlp_cmd_lower_addr_reg;
                    tlp_len_next = tlp_cmd_byte_len_reg;
                    dword_count_next = tlp_cmd_dword_len_reg;
                    offset_next = tlp_cmd_offset_reg;
                    input_cycle_count_next = tlp_cmd_input_cycle_len_reg;
                    output_cycle_count_next = tlp_cmd_output_cycle_len_reg;
                    input_active_next = 1'b1;
                    bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
                    last_cycle_next = tlp_cmd_output_cycle_len_reg == 0;
                    last_tlp_next = tlp_cmd_last_reg;
                    status_next = tlp_cmd_status_reg;
                    requester_id_next = tlp_cmd_requester_id_reg;
                    tag_next = tlp_cmd_tag_reg;
                    tc_next = tlp_cmd_tc_reg;
                    attr_next = tlp_cmd_attr_reg;

                    if (tlp_cmd_valid_reg) begin
                        tlp_cmd_ready = 1'b1;
                        m_axi_rready_next = 1'b0;
                        tlp_state_next = TLP_STATE_HEADER_1;
                    end else begin
                        m_axi_rready_next = 1'b0;
                        tlp_state_next = TLP_STATE_IDLE;
                    end
                end else begin
                    m_axi_rready_next = m_axis_cc_tready_int_early && input_active_next;
                    tlp_state_next = TLP_STATE_TRANSFER;
                end
            end else begin
                tlp_state_next = TLP_STATE_HEADER_2;
            end
        end

        TLP_STATE_TRANSFER: begin
            // transfer state, transfer data
            m_axi_rready_next = m_axis_cc_tready_int_early && input_active_reg;

            if (m_axis_cc_tready_int_reg && ((m_axi_rready && m_axi_rvalid) || !input_active_reg)) begin
                transfer_in_save = 1'b1;

                if (bubble_cycle_reg) begin
                    // bubble cycle; store input data and update input cycle count
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next = input_cycle_count_reg > 0;
                    end
                    bubble_cycle_next = 1'b0;
                    m_axi_rready_next = m_axis_cc_tready_int_early && input_active_next;
                    tlp_state_next = TLP_STATE_TRANSFER;
                end else begin
                    // update DWORD count
                    dword_count_next = dword_count_reg - AXI_STRB_WIDTH/4;
                    // update cycle counters
                    if (input_active_reg) begin
                        input_cycle_count_next = input_cycle_count_reg - 1;
                        input_active_next = input_cycle_count_reg > 0;
                    end
                    output_cycle_count_next = output_cycle_count_reg - 1;
                    last_cycle_next = output_cycle_count_next == 0;

                    // output data and generate tvalid and tkeep signals
                    m_axis_cc_tdata_int = shift_axi_rdata;
                    m_axis_cc_tvalid_int = 1'b1;
                    if (dword_count_reg >= AXI_STRB_WIDTH/4) begin
                        m_axis_cc_tkeep_int = {AXI_STRB_WIDTH{1'b1}};
                    end else begin
                        m_axis_cc_tkeep_int = {AXI_STRB_WIDTH{1'b1}} >> (AXI_STRB_WIDTH - dword_count_reg);
                    end

                    if (last_cycle_reg) begin
                        m_axis_cc_tlast_int = 1'b1;

                        // skip idle state if possible
                        tlp_lower_addr_next = tlp_cmd_lower_addr_reg;
                        tlp_len_next = tlp_cmd_byte_len_reg;
                        dword_count_next = tlp_cmd_dword_len_reg;
                        offset_next = tlp_cmd_offset_reg;
                        input_cycle_count_next = tlp_cmd_input_cycle_len_reg;
                        output_cycle_count_next = tlp_cmd_output_cycle_len_reg;
                        input_active_next = 1'b1;
                        bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
                        last_cycle_next = tlp_cmd_output_cycle_len_reg == 0;
                        last_tlp_next = tlp_cmd_last_reg;
                        status_next = tlp_cmd_status_reg;
                        requester_id_next = tlp_cmd_requester_id_reg;
                        tag_next = tlp_cmd_tag_reg;
                        tc_next = tlp_cmd_tc_reg;
                        attr_next = tlp_cmd_attr_reg;

                        if (tlp_cmd_valid_reg) begin
                            tlp_cmd_ready = 1'b1;
                            if (AXIS_PCIE_DATA_WIDTH == 64) begin
                                m_axi_rready_next = 1'b0;
                            end else begin
                                m_axi_rready_next = m_axis_cc_tready_int_early;
                            end
                            tlp_state_next = TLP_STATE_HEADER_1;
                        end else begin
                            m_axi_rready_next = 1'b0;
                            tlp_state_next = TLP_STATE_IDLE;
                        end
                    end else begin
                        m_axi_rready_next = m_axis_cc_tready_int_early && input_active_next;
                        tlp_state_next = TLP_STATE_TRANSFER;
                    end
                end
            end else begin
                tlp_state_next = TLP_STATE_TRANSFER;
            end
        end

        TLP_STATE_CPL_1: begin
            // send completion
            m_axis_cc_tvalid_int = 1'b1;
            m_axis_cc_tdata_int[28:16] = 13'd0; // byte count
            m_axis_cc_tdata_int[42:32] = 11'd0; // DWORD count
            m_axis_cc_tdata_int[45:43] = status_reg;

            // generate tvalid and tkeep signals for completion
            if (AXIS_PCIE_DATA_WIDTH == 512) begin
                m_axis_cc_tkeep_int = 16'b0000000000000111;
                m_axis_cc_tlast_int = 1'b1;
            end else if (AXIS_PCIE_DATA_WIDTH == 256) begin
                m_axis_cc_tkeep_int = 8'b00000111;
                m_axis_cc_tlast_int = 1'b1;
            end else if (AXIS_PCIE_DATA_WIDTH == 128) begin
                m_axis_cc_tkeep_int = 4'b0111;
                m_axis_cc_tlast_int = 1'b1;
            end else if (AXIS_PCIE_DATA_WIDTH == 64) begin
                m_axis_cc_tkeep_int = 2'b11;
                m_axis_cc_tlast_int = 1'b0;
            end

            if (m_axis_cc_tready_int_reg) begin
                if (AXIS_PCIE_DATA_WIDTH == 64) begin
                    tlp_state_next = TLP_STATE_CPL_2;
                end else begin
                    // skip idle state if possible
                    tlp_lower_addr_next = tlp_cmd_lower_addr_reg;
                    tlp_len_next = tlp_cmd_byte_len_reg;
                    dword_count_next = tlp_cmd_dword_len_reg;
                    offset_next = tlp_cmd_offset_reg;
                    input_cycle_count_next = tlp_cmd_input_cycle_len_reg;
                    output_cycle_count_next = tlp_cmd_output_cycle_len_reg;
                    input_active_next = 1'b1;
                    bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
                    last_cycle_next = tlp_cmd_output_cycle_len_reg == 0;
                    last_tlp_next = tlp_cmd_last_reg;
                    status_next = tlp_cmd_status_reg;
                    requester_id_next = tlp_cmd_requester_id_reg;
                    tag_next = tlp_cmd_tag_reg;
                    tc_next = tlp_cmd_tc_reg;
                    attr_next = tlp_cmd_attr_reg;

                    if (tlp_cmd_valid_reg) begin
                        tlp_cmd_ready = 1'b1;
                        m_axi_rready_next = m_axis_cc_tready_int_early;
                        tlp_state_next = TLP_STATE_HEADER_1;
                    end else begin
                        m_axi_rready_next = 1'b0;
                        tlp_state_next = TLP_STATE_IDLE;
                    end
                end
            end else begin
                tlp_state_next = TLP_STATE_CPL_1;
            end
        end

        TLP_STATE_CPL_2: begin
            // send rest of completion
            m_axis_cc_tvalid_int = 1'b1;
            m_axis_cc_tdata_int[7:0] = tag_reg;
            m_axis_cc_tdata_int[23:8] = completer_id;
            m_axis_cc_tdata_int[24] = completer_id_enable;
            m_axis_cc_tdata_int[27:25] = tc_reg;
            m_axis_cc_tdata_int[30:28] = attr_reg;
            m_axis_cc_tdata_int[31] = 1'b0; // force ECRC
            m_axis_cc_tdata_int[63:32] = 32'd0;
            m_axis_cc_tkeep_int = 2'b01;
            m_axis_cc_tlast_int = 1'b1;

            if (m_axis_cc_tready_int_reg) begin
                // skip idle state if possible
                tlp_lower_addr_next = tlp_cmd_lower_addr_reg;
                tlp_len_next = tlp_cmd_byte_len_reg;
                dword_count_next = tlp_cmd_dword_len_reg;
                offset_next = tlp_cmd_offset_reg;
                input_cycle_count_next = tlp_cmd_input_cycle_len_reg;
                output_cycle_count_next = tlp_cmd_output_cycle_len_reg;
                input_active_next = 1'b1;
                bubble_cycle_next = tlp_cmd_bubble_cycle_reg;
                last_cycle_next = tlp_cmd_output_cycle_len_reg == 0;
                last_tlp_next = tlp_cmd_last_reg;
                status_next = tlp_cmd_status_reg;
                requester_id_next = tlp_cmd_requester_id_reg;
                tag_next = tlp_cmd_tag_reg;
                tc_next = tlp_cmd_tc_reg;
                attr_next = tlp_cmd_attr_reg;

                if (tlp_cmd_valid_reg) begin
                    tlp_cmd_ready = 1'b1;
                    m_axi_rready_next = 1'b0;
                    tlp_state_next = TLP_STATE_HEADER_1;
                end else begin
                    m_axi_rready_next = 1'b0;
                    tlp_state_next = TLP_STATE_IDLE;
                end
            end else begin
                tlp_state_next = TLP_STATE_CPL_2;
            end
        end

        default: tlp_state_next = TLP_STATE_IDLE;        

    endcase
    end

end

always_ff @(posedge clk) begin
    if (rst) begin
        axi_state_reg <= AXI_STATE_IDLE;
        tlp_state_reg <= TLP_STATE_IDLE;
        tlp_cmd_valid_reg <= 1'b0;
        s_axis_cq_tready_reg <= 1'b0;
        m_axi_arvalid_reg <= 1'b0;
        m_axi_rready_reg <= 1'b0;

        status_error_cor_reg <= 1'b0;
        status_error_uncor_reg <= 1'b0;
    end else begin
        axi_state_reg <= axi_state_next;
        tlp_state_reg <= tlp_state_next;
        tlp_cmd_valid_reg <= tlp_cmd_valid_next;
        s_axis_cq_tready_reg <= s_axis_cq_tready_next;
        m_axi_arvalid_reg <= m_axi_arvalid_next;
        m_axi_rready_reg <= m_axi_rready_next;

        status_error_cor_reg <= status_error_cor_next;
        status_error_uncor_reg <= status_error_uncor_next;
    end

    axi_addr_reg <= axi_addr_next;
    op_count_reg <= op_count_next;
    op_dword_count_reg <= op_dword_count_next;
    tlp_dword_count_reg <= tlp_dword_count_next;
    first_be_reg <= first_be_next;
    last_be_reg <= last_be_next;

    tlp_lower_addr_reg <= tlp_lower_addr_next;
    tlp_len_reg <= tlp_len_next;
    dword_count_reg <= dword_count_next;
    offset_reg <= offset_next;
    input_cycle_count_reg <= input_cycle_count_next;
    output_cycle_count_reg <= output_cycle_count_next;
    input_active_reg <= input_active_next;
    bubble_cycle_reg <= bubble_cycle_next;
    last_cycle_reg <= last_cycle_next;
    last_tlp_reg <= last_tlp_next;
    status_reg <= status_next;
    requester_id_reg <= requester_id_next;
    tag_reg <= tag_next;
    tc_reg <= tc_next;
    attr_reg <= attr_next;

    tlp_cmd_lower_addr_reg <= tlp_cmd_lower_addr_next;
    tlp_cmd_byte_len_reg <= tlp_cmd_byte_len_next;
    tlp_cmd_dword_len_reg <= tlp_cmd_dword_len_next;
    tlp_cmd_input_cycle_len_reg <= tlp_cmd_input_cycle_len_next;
    tlp_cmd_output_cycle_len_reg <= tlp_cmd_output_cycle_len_next;
    tlp_cmd_offset_reg <= tlp_cmd_offset_next;
    tlp_cmd_status_reg <= tlp_cmd_status_next;
    tlp_cmd_requester_id_reg <= tlp_cmd_requester_id_next;
    tlp_cmd_tag_reg <= tlp_cmd_tag_next;
    tlp_cmd_tc_reg <= tlp_cmd_tc_next;
    tlp_cmd_attr_reg <= tlp_cmd_attr_next;
    tlp_cmd_bubble_cycle_reg <= tlp_cmd_bubble_cycle_next;
    tlp_cmd_last_reg <= tlp_cmd_last_next;

    m_axi_araddr_reg <= m_axi_araddr_next;
    m_axi_arlen_reg <= m_axi_arlen_next;

    max_payload_size_dw_reg <= 11'd32 << (max_payload_size > PAYLOAD_MAX ? PAYLOAD_MAX : max_payload_size);

    if (transfer_in_save) begin
        save_axi_rdata_reg <= m_axi_rdata;
    end    
end

// output datapath logic (PCIe TLP)
logic [AXIS_PCIE_DATA_WIDTH-1:0]    m_axis_cc_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
logic [AXIS_PCIE_KEEP_WIDTH-1:0]    m_axis_cc_tkeep_reg = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
logic                               m_axis_cc_tvalid_reg = 1'b0, m_axis_cc_tvalid_next;
logic                               m_axis_cc_tlast_reg = 1'b0;
logic [AXIS_PCIE_CC_USER_WIDTH-1:0] m_axis_cc_tuser_reg = {AXIS_PCIE_CC_USER_WIDTH{1'b0}};

logic [AXIS_PCIE_DATA_WIDTH-1:0]    temp_m_axis_cc_tdata_reg = {AXIS_PCIE_DATA_WIDTH{1'b0}};
logic [AXIS_PCIE_KEEP_WIDTH-1:0]    temp_m_axis_cc_tkeep_reg = {AXIS_PCIE_KEEP_WIDTH{1'b0}};
logic                               temp_m_axis_cc_tvalid_reg = 1'b0, temp_m_axis_cc_tvalid_next;
logic                               temp_m_axis_cc_tlast_reg = 1'b0;
logic [AXIS_PCIE_CC_USER_WIDTH-1:0] temp_m_axis_cc_tuser_reg = {AXIS_PCIE_CC_USER_WIDTH{1'b0}};

// datapath control
logic store_axis_cc_int_to_output;
logic store_axis_cc_int_to_temp;
logic store_axis_cc_temp_to_output;

assign m_axis_cc_tdata = m_axis_cc_tdata_reg;
assign m_axis_cc_tkeep = m_axis_cc_tkeep_reg;
assign m_axis_cc_tvalid = m_axis_cc_tvalid_reg;
assign m_axis_cc_tlast = m_axis_cc_tlast_reg;
assign m_axis_cc_tuser = m_axis_cc_tuser_reg;

// enable ready input next cycle if output is ready or if both output registers are empty
assign m_axis_cc_tready_int_early = m_axis_cc.tready || (!temp_m_axis_cc_tvalid_reg && !m_axis_cc_tvalid_reg);

always_comb begin
    // transfer sink ready state to source
    m_axis_cc_tvalid_next = m_axis_cc_tvalid_reg;
    temp_m_axis_cc_tvalid_next = temp_m_axis_cc_tvalid_reg;

    store_axis_cc_int_to_output = 1'b0;
    store_axis_cc_int_to_temp = 1'b0;
    store_axis_cc_temp_to_output = 1'b0;

    if (m_axis_cc_tready_int_reg) begin
        // input is ready
        if (m_axis_cc_tready || !m_axis_cc_tvalid_reg) begin
            // output is ready or currently not valid, transfer data to output
            m_axis_cc_tvalid_next = m_axis_cc_tvalid_int;
            store_axis_cc_int_to_output = 1'b1;
        end else begin
            // output is not ready, store input in temp
            temp_m_axis_cc_tvalid_next = m_axis_cc_tvalid_int;
            store_axis_cc_int_to_temp = 1'b1;
        end
    end else if (m_axis_cc_tready) begin
        // input is not ready, but output is ready
        m_axis_cc_tvalid_next = temp_m_axis_cc_tvalid_reg;
        temp_m_axis_cc_tvalid_next = 1'b0;
        store_axis_cc_temp_to_output = 1'b1;
    end
end

always_ff @(posedge clk) begin
    m_axis_cc_tvalid_reg <= m_axis_cc_tvalid_next;
    m_axis_cc_tready_int_reg <= m_axis_cc_tready_int_early;
    temp_m_axis_cc_tvalid_reg <= temp_m_axis_cc_tvalid_next;

    // datapath
    if (store_axis_cc_int_to_output) begin
        m_axis_cc_tdata_reg <= m_axis_cc_tdata_int;
        m_axis_cc_tkeep_reg <= m_axis_cc_tkeep_int;
        m_axis_cc_tlast_reg <= m_axis_cc_tlast_int;
        m_axis_cc_tuser_reg <= m_axis_cc_tuser_int;
    end else if (store_axis_cc_temp_to_output) begin
        m_axis_cc_tdata_reg <= temp_m_axis_cc_tdata_reg;
        m_axis_cc_tkeep_reg <= temp_m_axis_cc_tkeep_reg;
        m_axis_cc_tlast_reg <= temp_m_axis_cc_tlast_reg;
        m_axis_cc_tuser_reg <= temp_m_axis_cc_tuser_reg;
    end

    if (store_axis_cc_int_to_temp) begin
        temp_m_axis_cc_tdata_reg <= m_axis_cc_tdata_int;
        temp_m_axis_cc_tkeep_reg <= m_axis_cc_tkeep_int;
        temp_m_axis_cc_tlast_reg <= m_axis_cc_tlast_int;
        temp_m_axis_cc_tuser_reg <= m_axis_cc_tuser_int;
    end

    if (rst) begin
        m_axis_cc_tvalid_reg <= 1'b0;
        m_axis_cc_tready_int_reg <= 1'b0;
        temp_m_axis_cc_tvalid_reg <= 1'b0;
    end
end

endmodule
`resetall
