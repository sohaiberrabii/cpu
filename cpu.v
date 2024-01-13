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
    // register file
    reg [31:0] rf[31:0];

    // Memory stage signals
    reg [4:0] rd_m;
    reg [31:0] aluresult_m, pcplus4_m, wdata_m, immext_m;
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
    wire [31:0] instr_f, pcplus4_f;
    reg [31:0] pc_f;
    assign instr_f = instr;

    // Decode stage signals
    reg [31:0] instr_d, pcplus4_d, pc_d;
    wire [2:0] aluctl_d;
    wire [1:0] resultsrc_d;
    wire alusrc_d, regwrite_d, memwrite_d, nbranch_d, branch_d, jump_d;
    wire [12:0] controls_d;
    wire is_auipc_d, is_jalr_d;
    assign controls_d = {
        regwrite_d, resultsrc_d, memwrite_d, nbranch_d, branch_d,
        jump_d, aluctl_d, alusrc_d, is_auipc_d, is_jalr_d
    };

    // Execute stage signals
    wire is_auipc_e, is_jalr_e;
    reg [31:0] rs1d_e, rs2d_e;
    reg [4:0] rs1_e, rs2_e, rd_e;
    reg [31:0] pcplus4_e, pc_e, immext_e;
    wire [31:0] pctarget_e;
    wire [2:0] aluctl_e;
    wire [1:0] resultsrc_e;
    wire alusrc_e, regwrite_e, memwrite_e, nbranch_e, branch_e, jump_e, pcsrc_e;
    reg [12:0] controls_e;
    assign {
        regwrite_e, resultsrc_e, memwrite_e, nbranch_e, branch_e, jump_e, aluctl_e, alusrc_e,
        is_auipc_e, is_jalr_e
    } = controls_e;
    reg [31:0] aluresult_e;
    wire lt_e = aluresult_e[0]; // aluctl => less than
    wire zero_e = aluresult_e == 0; // aluctl => sub

    // Writeback stage signals
    reg [4:0] rd_w;
    reg [31:0] aluresult_w, pcplus4_w, immext_w;
    reg [31:0] result_w;
    wire regwrite_w;
    wire [1:0] resultsrc_w;
    reg [2:0] controls_w;
    assign {regwrite_w, resultsrc_w} = controls_w;

    // Hazard handling signals
    wire [1:0] forward1, forward2;
    wire flushd, flushe, stallf, stalld, forward_d;

    // Fetch
    assign pcsrc_e = (nbranch_e ? ~zero_e : zero_e) & branch_e | jump_e;
    assign pcplus4_f = pc_f + 4;

    always @(posedge clk) begin
        if (reset)
            pc_f <= 0;
        else if (~stallf)
            if (pcsrc_e)
                pc_f <= {pctarget_e[31:1], 1'b0};
            else
                pc_f <= pcplus4_f;
    end

    // Decode
    always @(posedge clk) begin
        if (reset | flushd)
            {instr_d, pc_d, pcplus4_d} <= 0;
        else if (~stalld)
            {instr_d, pc_d, pcplus4_d} <= {instr_f, pc_f, pcplus4_f};
    end

    controller ctl (
        .opcode(instr_d[6:0]), .funct3(instr_d[14:12]), .funct75(instr_d[30]),
        .alusrc(alusrc_d), .resultsrc(resultsrc_d),
        .branch(branch_d), .jump(jump_d), .memwrite(memwrite_d), .regwrite(regwrite_d),
        .aluctl(aluctl_d), .nbranch(nbranch_d), .is_auipc(is_auipc_d), .is_jalr(is_jalr_d)
    );

    wire instr_lui   = instr_d[6:0] == 7'b0110111;
    wire instr_auipc = instr_d[6:0] == 7'b0010111;
    wire instr_jal   = instr_d[6:0] == 7'b1101111;
    wire instr_jalr  = instr_d[6:0] == 7'b1100111 && instr_d[14:12] == 3'b000;

    wire is_beq_bne_blt_bge_bltu_bgeu = instr_d[6:0] == 7'b1100011;
    wire instr_beq   = is_beq_bne_blt_bge_bltu_bgeu && instr_d[14:12] == 3'b000;
    wire instr_bne   = is_beq_bne_blt_bge_bltu_bgeu && instr_d[14:12] == 3'b001;
    wire instr_blt   = is_beq_bne_blt_bge_bltu_bgeu && instr_d[14:12] == 3'b100;
    wire instr_bge   = is_beq_bne_blt_bge_bltu_bgeu && instr_d[14:12] == 3'b101;
    wire instr_bltu  = is_beq_bne_blt_bge_bltu_bgeu && instr_d[14:12] == 3'b110;
    wire instr_bgeu  = is_beq_bne_blt_bge_bltu_bgeu && instr_d[14:12] == 3'b111;

    wire is_lb_lh_lw_lbu_lhu    = instr_d[6:0] == 7'b0000011;
    wire instr_lb    = is_lb_lh_lw_lbu_lhu && instr_d[14:12] == 3'b000;
    wire instr_lh    = is_lb_lh_lw_lbu_lhu && instr_d[14:12] == 3'b001;
    wire instr_lw    = is_lb_lh_lw_lbu_lhu && instr_d[14:12] == 3'b010;
    wire instr_lbu   = is_lb_lh_lw_lbu_lhu && instr_d[14:12] == 3'b100;
    wire instr_lhu   = is_lb_lh_lw_lbu_lhu && instr_d[14:12] == 3'b101;

    wire is_sb_sh_sw    = instr_d[6:0] == 7'b0100011;
    wire instr_sb    = is_sb_sh_sw && instr_d[14:12] == 3'b000;
    wire instr_sh    = is_sb_sh_sw && instr_d[14:12] == 3'b001;
    wire instr_sw    = is_sb_sh_sw && instr_d[14:12] == 3'b010;

    wire is_alu_reg_imm = instr_d[6:0] == 7'b0010011;
    wire instr_addi  = is_alu_reg_imm && instr_d[14:12] == 3'b000;
    wire instr_slti  = is_alu_reg_imm && instr_d[14:12] == 3'b010;
    wire instr_sltiu = is_alu_reg_imm && instr_d[14:12] == 3'b011;
    wire instr_xori  = is_alu_reg_imm && instr_d[14:12] == 3'b100;
    wire instr_ori   = is_alu_reg_imm && instr_d[14:12] == 3'b110;
    wire instr_andi  = is_alu_reg_imm && instr_d[14:12] == 3'b111;

    wire instr_slli  = is_alu_reg_imm && instr_d[14:12] == 3'b001 && instr_d[31:25] == 7'b0000000;
    wire instr_srli  = is_alu_reg_imm && instr_d[14:12] == 3'b101 && instr_d[31:25] == 7'b0000000;
    wire instr_srai  = is_alu_reg_imm && instr_d[14:12] == 3'b101 && instr_d[31:25] == 7'b0100000;
    wire is_slli_srli_srai = |{instr_slli, instr_srli, instr_srai};

    wire is_alu_reg_reg = instr_d[6:0] == 7'b0110011;
    wire instr_add   = is_alu_reg_reg && instr_d[14:12] == 3'b000 && instr_d[31:25] == 7'b0000000;
    wire instr_sub   = is_alu_reg_reg && instr_d[14:12] == 3'b000 && instr_d[31:25] == 7'b0100000;
    wire instr_sll   = is_alu_reg_reg && instr_d[14:12] == 3'b001 && instr_d[31:25] == 7'b0000000;
    wire instr_slt   = is_alu_reg_reg && instr_d[14:12] == 3'b010 && instr_d[31:25] == 7'b0000000;
    wire instr_sltu  = is_alu_reg_reg && instr_d[14:12] == 3'b011 && instr_d[31:25] == 7'b0000000;
    wire instr_xor   = is_alu_reg_reg && instr_d[14:12] == 3'b100 && instr_d[31:25] == 7'b0000000;
    wire instr_srl   = is_alu_reg_reg && instr_d[14:12] == 3'b101 && instr_d[31:25] == 7'b0000000;
    wire instr_sra   = is_alu_reg_reg && instr_d[14:12] == 3'b101 && instr_d[31:25] == 7'b0100000;
    wire instr_or    = is_alu_reg_reg && instr_d[14:12] == 3'b110 && instr_d[31:25] == 7'b0000000;
    wire instr_and   = is_alu_reg_reg && instr_d[14:12] == 3'b111 && instr_d[31:25] == 7'b0000000;

    wire is_i_type = |{instr_jalr, is_lb_lh_lw_lbu_lhu, is_alu_reg_imm} && !is_slli_srli_srai;
    wire is_s_type = is_sb_sh_sw;
    wire is_b_type = is_beq_bne_blt_bge_bltu_bgeu;
    wire is_u_type = instr_lui || instr_auipc;
    wire is_j_type = instr_jal;

    // decode to execute
    always @(posedge clk) begin
        if (reset | flushe)
            {rs1d_e, rs2d_e, rs1_e, immext_e, rd_e} <= 0;
        else begin
            // register operands
            rs1_e <= instr_d[19:15];
            rs2_e <= instr_d[24:20];
            rs1d_e <= (instr_d[19:15] != 0) ? rf[instr_d[19:15]] : 0;
            rs2d_e <= (instr_d[24:20] != 0) ? rf[instr_d[24:20]] : 0;
            rd_e <= instr_d[11:7];

            // immediate
            (* parallel_case *)
            case (1'b1)
                is_i_type: immext_e <= {{20{instr_d[31]}}, instr_d[31:20]};
                is_s_type: immext_e <= {{20{instr_d[31]}}, instr_d[31:25], instr_d[11:7]};
                is_b_type: immext_e <= {{20{instr_d[31]}}, instr_d[7], instr_d[30:25], instr_d[11:8], 1'b0};
                is_j_type: immext_e <= {{12{instr_d[31]}}, instr_d[19:12], instr_d[20], instr_d[30:21], 1'b0};
                is_u_type: immext_e <= {instr_d[31:12], 12'b0};
                default: immext_e <= 32'bx;
            endcase
        end
    end

    // forward logic
    reg [31:0] src1, src2;
    always @(*) begin
        case(forward1)
           2'b00: src1 = rs1d_e;
           2'b01: src1 = result_w;
           2'b10: src1 = aluresult_m;
           2'b11: src1 = immext_m;
        endcase
        case(forward2)
           2'b00: src2 = rs2d_e;
           2'b01: src2 = result_w;
           2'b10: src2 = aluresult_m;
           2'b11: src2 = immext_m;
        endcase
    end

    // Execute
    wire [31:0] jumpsrc = is_jalr_e ? src1 : pc_e;
    assign pctarget_e = jumpsrc + immext_e;

    wire [31:0] a = is_auipc_e ? pc_e : src1;
    wire [31:0] b = alusrc_e ? immext_e : src2;
    wire [31:0] condnotb = aluctl_e[0] ? ~b : b;
    wire [31:0] add_sub = a + condnotb + aluctl_e[0]; // 2s comp
    always @(*) begin
        casez (aluctl_e)
            3'b00?: aluresult_e = add_sub;
            3'b010: aluresult_e = a & b;
            3'b011: aluresult_e = a | b;
            3'b100: aluresult_e = a ^ b;
            3'b101: aluresult_e = $signed(a) < $signed(b);
            3'b110: aluresult_e = a << b[4:0];
            3'b111: aluresult_e = a >> b[4:0];
            default: aluresult_e = 32'bx;
        endcase
    end

    // Writeback
    always @(*) case(resultsrc_w)
        2'b00: result_w = aluresult_w;
        2'b01: result_w = rdata_w;
        2'b10: result_w = pcplus4_w;
        2'b11: result_w = immext_w; // LUI
    endcase
    always @(negedge clk)
        if (regwrite_w) rf[rd_w] <= result_w;

    hazard hzd (
        .regwrite_m(regwrite_m), .regwrite_w(regwrite_w),
        .rs1_e(rs1_e), .rs2_e(rs2_e), .rd_m(rd_m), .rd_w(rd_w),
        .rs1_d(instr_d[19:15]), .rs2_d(instr_d[24:20]), .rd_e(rd_e),
        .forward1(forward1), .forward2(forward2),
        .pcsrc_e(pcsrc_e), .flushd(flushd), .flushe(flushe),
        .stallf(stallf), .stalld(stalld), .resultsrc_e(resultsrc_e), .resultsrc_m(resultsrc_m)
    );

    always @(posedge clk) begin
        // flush all signals from pipeline at reset
        // TODO: this is disgusting and not actually needed? isn't it enough to deassert memwrite/regwrite
        if (reset) begin
            {pcplus4_e, pc_e} <= 0;
            {rd_m, pcplus4_m, wdata_m, aluresult_m, immext_m} <= 0;
            {rd_w, pcplus4_w, aluresult_w, immext_w} <= 0;
            {controls_e, controls_m, controls_w} <= 0;
        end else begin
            if (flushe)
                {pcplus4_e, pc_e, controls_e} <= 0;
            else begin
                {pcplus4_e, pc_e} <= {pcplus4_d, pc_d};
                controls_e <= controls_d;
            end

            {rd_m, pcplus4_m, wdata_m, aluresult_m, immext_m} <= {rd_e, pcplus4_e, src2, aluresult_e, immext_e};
            {rd_w, pcplus4_w, aluresult_w, immext_w} <= {rd_m, pcplus4_m, aluresult_m, immext_m};
            {controls_m, controls_w} <= {controls_e[12:9], controls_m[3:1]};
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
        if (resultsrc_m == 2'b11 & rs1_e == rd_m & rs1_e != 0)
            forward1 = 2'b11;
        else if ((regwrite_m & rs1_e == rd_m) & rs1_e != 0)
            forward1 = 2'b10;
        else if ((regwrite_w & rs1_e == rd_w) & rs1_e != 0)
            forward1 = 2'b01;
        else
            forward1 = 2'b00;

        if (resultsrc_m == 2'b11 & rs2_e == rd_m & rs2_e != 0)
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

module controller (
    input [6:0] opcode, input [2:0] funct3, input funct75, // funct7[5]
    output nbranch, branch, jump, alusrc, regwrite, memwrite,
    output [1:0] resultsrc,
    output [2:0] aluctl,
    output is_auipc, is_jalr
);
    reg [11:0] controls;
    assign {aluctl, resultsrc, alusrc, regwrite, memwrite, branch, jump, nbranch, is_auipc} = controls;

    // TODO: handle is_auipc, nbranch similarly
    assign is_jalr = jump & ~opcode[3];

    always @(*) begin
        casez (opcode)
            // aluctl_resultsrc_alusrc_regwrite_memwrite_branch_jump_nbranch_isauipc
            7'b0000011: controls = 12'b000_01_1_1_0_0_0_0_0; // lw
            7'b0100011: controls = 12'b000_00_1_0_1_0_0_0_0; // sw
            7'b1100011: controls = {10'b001_00_0_0_0_1_0, funct3[0], 1'b0}; // beq, bne
            7'b1101111: controls = 12'b000_10_0_1_0_0_1_0_0; // jal
            7'b1100111: controls = 12'b000_10_0_1_0_0_1_0_0; // jalr
            7'b0110111: controls = 12'b000_11_0_1_0_0_0_0_0; // lui
            7'b0010111: controls = 12'b000_00_1_1_0_0_0_0_1; // auipc
            7'b0?10011: begin // R-type or I-type ALU
                controls[8:0] = opcode[5] ? 9'bxx_0_1_0_0_0_0_0 : 9'b00_1_1_0_0_0_0_0;
                case (funct3)
                    3'b000: controls[11:9] = (funct75 & opcode[5]) ? 3'b001 : 3'b000; // sub,add,addi
                    3'b010: controls[11:9] = 3'b101; // slt, slti
                    3'b110: controls[11:9] = 3'b011; // or, ori
                    3'b111: controls[11:9] = 3'b010; // and, andi
                    default: controls[11:9] = 3'bxxx; // unassigned alu controls
                endcase
            end
            default: controls = 12'bx;
        endcase
    end
endmodule

