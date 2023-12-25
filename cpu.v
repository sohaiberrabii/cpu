`default_nettype none

module cpu(
    input clk, reset,
    output [31:0] mem_addr,     // data address bus
    output [31:0] mem_wdata,    // data to be written
    output mem_write,           // asserted to write to data memory
    input  [31:0] mem_rdata,    // input lines for both data and instr
    input  [31:0] instr,        // instruction data
    output [31:0] pc            // program counter for external instruction memory
);
    // Memory stage signals
    reg [4:0] rd_m;
    reg [31:0] aluresult_m, pcplus4_m, wdata_m, uimm_m;
    wire regwrite_m, memwrite_m;
    wire [1:0] resultsrc_m;
    reg [3:0] controls_m;
    assign {regwrite_m, resultsrc_m, memwrite_m} = controls_m;

    // with synchronous data memory there is no more memory stage rdata
    wire [31:0] rdata_w = mem_rdata;

    assign mem_addr = aluresult_m;
    assign mem_write = memwrite_m;
    assign mem_wdata = wdata_m;
    assign pc = pc_f;

    // Fetch stage signals
    wire [31:0] instr_f, pcplus4_f, pc_f;
    assign instr_f = instr;

    // Decode stage signals
    reg [31:0] instr_d, pcplus4_d, pc_d;
    wire [31:0] rs1d_d, rs2d_d, immext_d, uimm_d;
    wire [2:0] aluctl_d;
    wire [1:0] immsrc_d, resultsrc_d;
    wire alusrc_d, regwrite_d, memwrite_d, branch_d, jump_d;
    wire [11:0] controls_d;
    assign controls_d = {
        regwrite_d, resultsrc_d, memwrite_d, branch_d, jump_d, aluctl_d, alusrc_d, immsrc_d
    };

    // Execute stage signals
    reg [31:0] rs1d_e, rs2d_e;
    reg [4:0] rs1_e, rs2_e, rd_e;
    reg [31:0] pcplus4_e, pc_e, immext_e, uimm_e;
    wire [31:0] aluresult_e, pctarget_e;
    wire [2:0] aluctl_e;
    wire [1:0] resultsrc_e;
    wire zero_e, alusrc_e, regwrite_e, memwrite_e, branch_e, jump_e, pcsrc_e;
    reg [9:0] controls_e;
    assign {
        regwrite_e, resultsrc_e, memwrite_e, branch_e, jump_e, aluctl_e, alusrc_e
    } = controls_e;

    // Writeback stage signals
    reg [4:0] rd_w;
    reg [31:0] aluresult_w, pcplus4_w, uimm_w;
    wire [31:0] result_w;
    wire regwrite_w;
    wire [1:0] resultsrc_w;
    reg [2:0] controls_w;
    assign {regwrite_w, resultsrc_w} = controls_w;

    // Hazard handling signals
    wire [1:0] forward1, forward2;
    wire flushd, flushe, stallf, stalld, forward_d;

    assign pcsrc_e = zero_e & branch_e | jump_e;
    fetch fetch (
        .clk(clk), .ce(~stallf), .reset(reset), .pcsrc_e(pcsrc_e),
        .pctarget_e(pctarget_e),
        .pcplus4(pcplus4_f), .pc(pc_f)
    );

    decode decode (
        .clk(clk), .instr_d(instr_d),
        .regwrite_w(regwrite_w), .result_w(result_w),
        .rd_w(rd_w), .immsrc_d(immsrc_d),
        .rs1d(rs1d_d), .rs2d(rs2d_d), .immext(immext_d), .uimm(uimm_d)
    );

    // forward logic
    reg [31:0] src1, src2;
    always @(*) begin
        case(forward1)
           2'b00: src1 = rs1d_e;
           2'b01: src1 = result_w;
           2'b10: src1 = aluresult_m;
           2'b11: src1 = uimm_m;
        endcase
        case(forward2)
           2'b00: src2 = rs2d_e;
           2'b01: src2 = result_w;
           2'b10: src2 = aluresult_m;
           2'b11: src2 = uimm_m;
        endcase
    end

    execute execute(
        .src1(src1), .src2(src2),
        .pc_e(pc_e), .immext_e(immext_e), .alusrc_e(alusrc_e), .aluctl_e(aluctl_e),
        .zero(zero_e), .aluresult(aluresult_e), .pctarget(pctarget_e)
    );

    writeback writeback (
        .resultsrc_w(resultsrc_w),
        .aluresult_w(aluresult_w), .pcplus4_w(pcplus4_w), .rdata_w(rdata_w),
        .result(result_w), .uimm_w(uimm_w)
    );

    controller ctl (
        .opcode(instr_d[6:0]), .funct3(instr_d[14:12]), .funct75(instr_d[30]),
        .alusrc(alusrc_d), .immsrc(immsrc_d), .resultsrc(resultsrc_d),
        .branch(branch_d), .jump(jump_d), .memwrite(memwrite_d), .regwrite(regwrite_d),
        .aluctl(aluctl_d)
    );

    hazard hzd (
        .regwrite_m(regwrite_m), .regwrite_w(regwrite_w),
        .rs1_e(rs1_e), .rs2_e(rs2_e), .rd_m(rd_m), .rd_w(rd_w),
        .rs1_d(instr_d[19:15]), .rs2_d(instr_d[24:20]), .rd_e(rd_e),
        .forward1(forward1), .forward2(forward2),
        .pcsrc_e(pcsrc_e), .flushd(flushd), .flushe(flushe),
        .stallf(stallf), .stalld(stalld), .resultsrc_e(resultsrc_e), .resultsrc_m(resultsrc_m)
    );

    // datapath signals
    always @(posedge clk, posedge reset) begin
        // flush all signals from pipeline at reset
        // TODO: this is disgusting and not actually needed? isn't it enough to deassert memwrite/regwrite
        if (reset) begin
            {instr_d, pc_d, pcplus4_d} <= 96'b0;
            {rs1d_e, rs2d_e, rs1_e, rs2_e, rd_e, pcplus4_e, pc_e, immext_e, uimm_e} <= 207'b0;
            {rd_m, pcplus4_m, wdata_m, aluresult_m, uimm_m} <= 133'b0;
            {rd_w, pcplus4_w, aluresult_w, uimm_w} <= 101'b0;
            {controls_e, controls_m, controls_w} <= 16'b0;
        end else begin
            if (flushd)
                {instr_d, pc_d, pcplus4_d} <= 96'b0;
            else if (~stalld)
                {instr_d, pc_d, pcplus4_d} <= {instr_f, pc_f, pcplus4_f};

            if (flushe)
                {rs1d_e, rs2d_e, rs1_e, rs2_e, rd_e, pcplus4_e, pc_e, immext_e, uimm_e} <= 175'b0;
            else
                {rs1d_e, rs2d_e, rs1_e, rs2_e, rd_e, pcplus4_e, pc_e, immext_e, uimm_e} <= {
                        rs1d_d, rs2d_d, instr_d[19:15], instr_d[24:20], instr_d[11:7],
                        pcplus4_d, pc_d, immext_d, uimm_d
                };

            {rd_m, pcplus4_m, wdata_m, aluresult_m, uimm_m} <= {rd_e, pcplus4_e, src2, aluresult_e, uimm_e};
            {rd_w, pcplus4_w, aluresult_w, uimm_w} <= {rd_m, pcplus4_m, aluresult_m, uimm_m};
            {controls_e, controls_m, controls_w} <= {
                controls_d[11:2], controls_e[9:6], controls_m[3:1]
            };
        end
    end
endmodule

module hazard (
    input regwrite_m, regwrite_w,
    input [4:0] rs1_d, rs2_d, rs1_e, rs2_e, rd_e, rd_m, rd_w,
    output reg [1:0] forward1, forward2,
    input [1:0] resultsrc_e, resultsrc_m,
    input pcsrc_e,
    output stalld, stallf,
    output flushd, flushe
);
    // forward
    always @(*) begin
        if (resultsrc_m == 2'b11 & rs1_e == rd_m)
            forward1 = 2'b11;
        else if ((regwrite_m & rs1_e == rd_m) & rs1_e != 0)
            forward1 = 2'b10;
        else if ((regwrite_w & rs1_e == rd_w) & rs1_e != 0)
            forward1 = 2'b01;
        else
            forward1 = 2'b00;

        if (resultsrc_m == 2'b11 & rs2_e == rd_m)
            forward2 = 2'b11;
        else if ((regwrite_m & rs2_e == rd_m) & rs2_e != 0)
            forward2 = 2'b10;
        else if ((regwrite_w & rs2_e == rd_w) & rs2_e != 0)
            forward2 = 2'b01;
        else
            forward2 = 2'b00;
    end

    // stall
    wire lwstall = (rd_e == rs1_d | rd_e == rs2_d) & (resultsrc_e == 2'b01);
    assign {stallf, stalld} = {2{lwstall}};

    // flush
    assign flushd = pcsrc_e;
    assign flushe = pcsrc_e | lwstall;
endmodule

module fetch(
    input clk, ce, reset,
    input pcsrc_e,
    input [31:0] pctarget_e,
    output [31:0] pcplus4,
    output reg [31:0] pc
);
    assign pcplus4 = pc + 4;
    always @(posedge clk) begin
        if (reset)
            pc <= 0;
        else if (ce)
        // else if(ce & clkdiv_pulse)
            // 'if' not equivalent to ternary op with 'x valued signals (cf. IEEE-1364-2005).
            if (pcsrc_e) pc <= pctarget_e; else pc <= pcplus4;
    end
endmodule

module decode (
    input clk,
    input [31:0] instr_d,
    input regwrite_w,
    input [31:0] result_w,
    input [4:0] rd_w,
    input [1:0] immsrc_d,
    output [31:0] rs1d, rs2d, immext, uimm
);
    register_file reg_file (
        .clk(clk), .we3(regwrite_w),
        .addr1(instr_d[19:15]), .addr2(instr_d[24:20]), .addr3(rd_w),
        .rd1(rs1d), .rd2(rs2d), .wd3(result_w)
    );

    wire is_lui = instr_d[6:0] == 7'b0110111;
    assign uimm = {instr_d[31:12], 12'b0};

    extend ext (.instr(instr_d[31:7]), .immsrc(immsrc_d), .immext(immext));
endmodule

module execute (
    input [31:0] src1, src2,
    input [31:0] pc_e, immext_e,
    input alusrc_e,
    input [2:0] aluctl_e,
    output zero,
    output [31:0] aluresult,
    output [31:0] pctarget
);
    assign pctarget = pc_e + immext_e;
    alu alu (
        .a(src1), .b(alusrc_e ? immext_e : src2),
        .ctl(aluctl_e), .res(aluresult), .zero(zero)
    );
endmodule

module writeback (
    input [1:0] resultsrc_w,
    input [31:0] aluresult_w, pcplus4_w, rdata_w, uimm_w,
    output reg [31:0] result
);
    always @(*) case(resultsrc_w)
        2'b00: result = aluresult_w;
        2'b01: result = rdata_w;
        2'b10: result = pcplus4_w;
        2'b11: result = uimm_w; // U-type immediate (LUI, AUIPC)
    endcase
endmodule

module controller (
    input [6:0] opcode, input [2:0] funct3, input funct75, // funct7[5]
    output branch, jump, alusrc, regwrite, memwrite,
    output [1:0] resultsrc, immsrc,
    output [2:0] aluctl
);
    reg [11:0] controls;
    assign {aluctl, immsrc, resultsrc, alusrc, regwrite, memwrite, branch, jump} = controls;

    always @(*) begin
        casez (opcode)
            // aluctl_immsrc_resultsrc_alusrc_regwrite_memwrite_branch_jump
            7'b0000011: controls = 12'b000_00_01_1_1_0_0_0; // lw
            7'b0100011: controls = 12'b000_01_00_1_0_1_0_0; // sw
            7'b1100011: controls = 12'b001_10_00_0_0_0_1_0; // beq
            7'b1101111: controls = 12'b000_11_10_0_1_0_0_1; // jal
            7'b0110111: controls = 12'b000_xx_11_0_1_0_0_0; // lui
            7'b0?10011: begin // R-type or I-type ALU
                controls[8:0] = opcode[5] ? 9'bxx_00_0_1_0_0_0 : 9'b00_00_1_1_0_0_0;
                case (funct3)
                    3'b000: controls[11:9] = (funct75 & opcode[5]) ? 3'b001 : 3'b000; // sub,add,addi
                    3'b010: controls[11:9] = 3'b101; // slt, slti
                    3'b110: controls[11:9] = 3'b011; // or, ori
                    3'b111: controls[11:9] = 3'b010; // and, andi
                    default: controls[11:9] = 3'bxxx; // unassigned alu controls
                endcase
            end
            default: controls = 12'bxxx_xx_xx_x_x_x_x_x; // not implemented
        endcase
    end
endmodule

module extend (input [31:7] instr, input [1:0] immsrc, output reg [31:0] immext);
    always @(*)
        case (immsrc)
            2'b00: immext = {{20{instr[31]}}, instr[31:20]}; // I-type
            // S-type: sw immextsrc = 01
            2'b01: immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // S-type
            // B-type: beq immextsrc = 10
            2'b10: immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; // B-type
            2'b11: immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type
        endcase
endmodule

module alu (input [31:0] a, b, input [2:0] ctl, output reg [31:0] res, output zero);
    wire [31:0] condnotb = ctl[0] ? ~b : b;
    wire [31:0] sum = a + condnotb + ctl[0]; // 2s comp

    // TODO: check if abc can optimize this
    // isadd: 000, issub: 001 or 101
    wire isadd = ~ctl[0] & ~ctl[1] & ~ctl[2];
    wire issub = ctl[0] & ~ctl[1];
    wire ovf = ~(a[31] ^ b[31]) & (a[31] ^ sum[31]) & isadd |
                (a[31] ^ b[31]) & (a[31] ^ sum[31]) & issub;

    // TODO: optimize shifters to reduce lut count
    always @(*) begin
        casez (ctl)
            3'b00?: res = sum;
            3'b010: res = a & b;
            3'b011: res = a | b;
            3'b100: res = a ^ b;
            3'b101: res = sum[31] ^ ovf; // a-b<0 + no ovf or ovf negative side (a<0, b>0)
            3'b110: res = a << b[4:0];
            3'b111: res = a >> b[4:0];
            default: res = 32'bx;
        endcase
    end

    assign zero = res == 32'b0;
endmodule

module register_file (
    input clk, we3,
    input [4:0] addr1, addr2, addr3,
    input [31:0] wd3,
    output [31:0] rd1, rd2
);
    reg [31:0] rf[31:0];
    always @(negedge clk)
        if (we3) rf[addr3] <= wd3;

    assign rd1 = (addr1 != 0) ? rf[addr1] : 0;
    assign rd2 = (addr2 != 0) ? rf[addr2] : 0;
endmodule