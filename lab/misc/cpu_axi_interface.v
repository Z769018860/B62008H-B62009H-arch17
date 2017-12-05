`define INST_FIRST
// `define ALTERNATING
// `define CUSTOM_SEQUENCE
// `define SEQ              4'b1001
module cpu_axi_interface
(
    input              clk,
    input              resetn, 

    // inst SRAM-like 
    input              inst_req,
    input              inst_wr,
    input       [1 :0] inst_size,
    input       [31:0] inst_addr,
    input       [31:0] inst_wdata,
    output wire [31:0] inst_rdata,
    output wire        inst_addr_ok,
    output wire        inst_data_ok,
    
    // data SRAM-like 
    input              data_req,
    input              data_wr,
    input       [1 :0] data_size,
    input       [31:0] data_addr,
    input       [31:0] data_wdata,
    output wire [31:0] data_rdata,
    output wire        data_addr_ok,
    output wire        data_data_ok,

    // ar
    output wire [3 :0] arid,
    output wire [31:0] araddr,
    output wire [7 :0] arlen,
    output wire [2 :0] arsize,
    output wire [1 :0] arburst,
    output wire [1 :0] arlock,
    output wire [3 :0] arcache,
    output wire [2 :0] arprot,
    output wire        arvalid,
    input              arready,
    // r           
    input       [3 :0] rid,
    input       [31:0] rdata,
    input       [1 :0] rresp,
    input              rlast,
    input              rvalid,
    output wire        rready,
    // aw               
    output wire [3 :0] awid,
    output wire [31:0] awaddr,
    output wire [7 :0] awlen,
    output wire [2 :0] awsize,
    output wire [1 :0] awburst,
    output wire [1 :0] awlock,
    output wire [3 :0] awcache,
    output wire [2 :0] awprot,
    output wire        awvalid,
    input              awready,
    // w               
    output wire [3 :0] wid,
    output wire [31:0] wdata,
    output wire [3 :0] wstrb,
    output wire        wlast,
    output wire        wvalid,
    input              wready,
    // b                
    input       [3 :0] bid,
    input       [1 :0] bresp,
    input              bvalid,
    output wire        bready
);

    wire [66:0] inst_packet_i, inst_packet_o;
    wire [66:0] data_packet_i, data_packet_o;
    wire [ 3:0] inst_FIFO_cnt, data_FIFO_cnt;
    wire        inst_FIFO_full, data_FIFO_full;
    wire        inst_FIFO_empty, data_FIFO_empty;
    wire        inst_rd, data_rd;

    assign inst_packet_i = {inst_wr, inst_size, inst_addr, inst_wdata};
    assign data_packet_i = {data_wr, data_size, data_addr, data_wdata};

    // FIFO
    FIFO_queue inst_FIFO
    (
        .clk(clk),
        .srst(~resetn),
        .full(inst_FIFO_full),
        .din(inst_packet_i),
        .wr_en(inst_req && !inst_FIFO_full),
        .empty(inst_FIFO_empty),
        .dout(inst_packet_o),
        .rd_en(inst_rd)
        ,.data_count(inst_FIFO_cnt)
    );

    FIFO_queue data_FIFO
    (
        .clk(clk),
        .srst(~resetn),
        .full(data_FIFO_full),
        .din(data_packet_i),
        .wr_en(data_req && !data_FIFO_full),
        .empty(data_FIFO_empty),
        .dout(data_packet_o),
        .rd_en(data_rd)
        ,.data_count(data_FIFO_cnt)
    );

    assign inst_addr_ok = !inst_FIFO_full;
    assign data_addr_ok = !data_FIFO_full;

    // SRAM-like
    assign inst_rdata = rdata;
    assign data_rdata = rdata;

    // FSM transaction control
    parameter ARBIT = 4'd0;
    parameter DATA_W_BEGIN = 4'd1;
    parameter DATA_W_ADDR_READY = 4'd2;
    parameter DATA_W_DATA_READY = 4'd3;
    parameter DATA_R_BEGIN = 4'd4;
    parameter DATA_R_ADDR_READY = 4'd5;
    parameter INST_R_BEGIN = 4'd6;
    parameter INST_R_ADDR_READY = 4'd7;
    parameter ISSUE_DATA_DATA_OK = 4'd8;
    parameter ISSUE_INST_DATA_OK = 4'd9;
    parameter IDLE = 4'd10;

    reg [3:0] state, nextstate;
    always @(posedge clk)
        if (~resetn)
            state <= IDLE;
        else
            state <= nextstate;
    
    wire [3:0] sram_wen;
    decode_sram_wen dec(
        .size(     awsize),
        .addr(awaddr[1:0]),
        .wen (   sram_wen)
    );

    wire inst_pending, data_pending;
    `ifdef INST_FIRST
    // handle pending inst packets first
    assign inst_pending = !inst_FIFO_empty;
    assign data_pending = inst_FIFO_empty && !data_FIFO_empty;
    `endif

    assign inst_rd = (state == IDLE) ? inst_pending : 1'b0;
    assign data_rd = (state == IDLE) ? data_pending : 1'b0;

    assign data_data_ok = state == ISSUE_DATA_DATA_OK;
    assign inst_data_ok = state == ISSUE_INST_DATA_OK;
    
    reg arbit_inst, arbit_data;
    always @(posedge clk)
        if (state == IDLE) begin
            arbit_inst <= inst_rd;
            arbit_data <= data_rd;
        end

    always @(*) begin
        if (!resetn)
            nextstate = IDLE;
        else begin
            case (state)
                IDLE:
                    if (inst_pending || data_pending) nextstate = ARBIT;
                    else nextstate = IDLE;
                ARBIT: 
                    if (arbit_inst) nextstate = INST_R_BEGIN;   // inst master is read-only
                    else if (arbit_data) nextstate = data_packet_o[66] ? DATA_W_BEGIN : DATA_R_BEGIN;
                    else nextstate = ARBIT;
                DATA_W_BEGIN: 
                    nextstate = awready ? DATA_W_ADDR_READY : DATA_W_BEGIN;
                DATA_W_ADDR_READY: 
                    nextstate = wready ? DATA_W_DATA_READY : DATA_W_ADDR_READY;
                DATA_W_DATA_READY: 
                    nextstate = bvalid ? ISSUE_DATA_DATA_OK : DATA_W_DATA_READY;
                DATA_R_BEGIN: 
                    nextstate = arready ? DATA_R_ADDR_READY : DATA_R_BEGIN;
                DATA_R_ADDR_READY: 
                    nextstate = rvalid ? ISSUE_DATA_DATA_OK : DATA_R_ADDR_READY;
                INST_R_BEGIN: 
                    nextstate = arready ? INST_R_ADDR_READY : INST_R_BEGIN;
                INST_R_ADDR_READY: 
                    nextstate = rvalid ? ISSUE_INST_DATA_OK : INST_R_ADDR_READY;
                ISSUE_DATA_DATA_OK:
                    nextstate = IDLE;
                ISSUE_INST_DATA_OK: 
                    nextstate = IDLE;
                default: nextstate = IDLE;
            endcase
        end
    end

    // AXI Interface
    wire [66:0] read_packet;
    assign read_packet = ({67{state == DATA_R_BEGIN}} & data_packet_o) |
                         ({67{state == INST_R_BEGIN}} & inst_packet_o);
    // ar
    assign araddr  = read_packet[63:32];
    assign arid    = 4'd0;
    assign arlen   = 8'd0;
    assign arburst = 2'b01;
    assign arlock  = 2'd0;
    assign arcache = 4'd0;
    assign arprot  = 3'd0;
    assign arsize  = ({3{read_packet[65:64] == 2'b00}} & 3'd1) |
                     ({3{read_packet[65:64] == 2'b01}} & 3'd2) |
                     ({3{read_packet[65:64] == 2'b10}} & 3'd4);
    assign arvalid = state == INST_R_BEGIN || state == DATA_R_BEGIN;
    // r
    assign rready  = 1'b1;
    /* AXI spec 3.3: "While it is acceptable to wait for VALID
     to be asserted before asserting READY, it is also acceptable
     to assert READY by default prior to the assertion of VALID
     and this can result in a more efficient design."          */

    // aw
    assign awaddr  = data_packet_o[63:32];
    assign awid    = 4'd0; 
    assign awlen   = 8'd0;
    assign awburst = 2'b01;
    assign awlock  = 2'd0;
    assign awcache = 4'd0;
    assign awprot  = 3'd0;
    assign awvalid = (state == DATA_W_BEGIN) ? 1'b1 : 1'b0;
    assign awsize  = ({3{data_packet_o[65:64] == 2'b00}} & 3'd1) |
                     ({3{data_packet_o[65:64] == 2'b01}} & 3'd2) |
                     ({3{data_packet_o[65:64] == 2'b10}} & 3'd4);
    // w
    assign wid    = 4'd0;
    assign wdata  = data_packet_o[31:0];
    assign wstrb  = sram_wen;
    assign wlast  = 1'b1;
    assign wvalid = state == DATA_W_ADDR_READY;
    // b
    assign bready = 1'b1;
    /* same as rready */

endmodule

module decode_sram_wen(
    input  [2:0] size,
    input  [1:0] addr,

    output [3:0] wen
);

    assign wen = ({32{addr == 2'b00 && size == 3'd1}} & 4'b0001) |
                 ({32{addr == 2'b01 && size == 3'd1}} & 4'b0010) |
                 ({32{addr == 2'b10 && size == 3'd1}} & 4'b0100) |
                 ({32{addr == 2'b11 && size == 3'd1}} & 4'b1000) |
                 ({32{addr == 2'b00 && size == 3'd2}} & 4'b0011) |
                 ({32{addr == 2'b10 && size == 3'd2}} & 4'b1100) |
                 ({32{addr == 2'b00 && size == 3'd4}} & 4'b1111) |
                 // 3-byte write transactions
                 ({32{addr == 2'b01 && size == 3'd4}} & 4'b1110) |
                 ({32{addr == 2'b10 && size == 3'd4}} & 4'b0111);

endmodule
