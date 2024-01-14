`default_nettype none

module cpu(
    input clk, reset,
    output [31:0] mem_addr,
    output [31:0] mem_wdata,
    output mem_write,
    input  [31:0] mem_rdata,
    input  [31:0] instr,
    output [31:0] pc
);
    // register file
    reg [31:0] rf[31:0];

    wire [31:0] rdata_w = mem_rdata;
    assign mem_addr  = aluresult_m;
    assign mem_write = memwrite_m;
    assign mem_wdata = wdata_m;
    assign pc        = pc_f;

    // memory stage signals
    reg [4:0] rd_m;
    reg [31:0] aluresult_m, wdata_m;

    // fetch
    wire [31:0] instr_f = instr;
    reg [31:0] pc_f;
    always @(posedge clk) begin
        if (reset)
            pc_f <= 0;
        else if (~stallf)
            if (pcsrc_e)
                pc_f <= {pctarget_e[31:1], 1'b0};
            else
                pc_f <= pc_f + 4;
    end

    // decode
    reg [31:0] instr_d, pc_d;

    always @(posedge clk) begin
        if (reset | flushd)
            {instr_d, pc_d} <= 0;
        else if (~stalld)
            {instr_d, pc_d} <= {instr_f, pc_f};
    end

    // instructions
    wire instr_lui   = instr_d[6:0] == 7'b0110111;
    wire instr_auipc = instr_d[6:0] == 7'b0010111;
    wire instr_jal   = instr_d[6:0] == 7'b1101111;
    wire instr_jalr  = instr_d[6:0] == 7'b1100111 && instr_d[14:12] == 3'b000;
    wire is_lui_auipc_jal_jalr = |{instr_lui, instr_auipc, instr_jal, instr_jalr};

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

    // immediate types
    wire is_i_type = |{instr_jalr, is_lb_lh_lw_lbu_lhu, is_alu_reg_imm} && !is_slli_srli_srai;
    wire is_s_type = is_sb_sh_sw;
    wire is_b_type = is_beq_bne_blt_bge_bltu_bgeu;
    wire is_u_type = instr_lui || instr_auipc;
    wire is_j_type = instr_jal;

    // control lines for load/store
    reg memwrite_e, regwrite_e;
    reg memwrite_m, regwrite_m;
    reg memwrite_w, regwrite_w;
    reg is_load_e, is_load_m, is_load_w;
    always @(posedge clk) begin
        if (reset || flushe)
            {memwrite_e, regwrite_e} <= 0;
        else
            regwrite_e <= |{is_lui_auipc_jal_jalr, is_lb_lh_lw_lbu_lhu, is_alu_reg_imm, is_alu_reg_reg};
            memwrite_e <= is_sb_sh_sw;

            regwrite_m <= regwrite_e;
            memwrite_m <= memwrite_e;

            regwrite_w <= regwrite_m;

            is_load_e <= is_lb_lh_lw_lbu_lhu;
            is_load_m <= is_load_e;
            is_load_w <= is_load_m;
    end

    // alu ops
    reg alu_add;
    reg alu_sub;
    reg alu_shl;
    reg alu_lt;
    reg alu_ltu;
    reg alu_xor;
    reg alu_shrl;
    reg alu_shra;
    reg alu_or;
    reg alu_and;

    reg alu_eq;
    reg alu_neq;
    reg alu_ge;
    reg alu_geu;

    reg alu_0_op1;
    reg alu_pc_op1;
    reg alu_shamt_op2;
    reg alu_imm_op2;

    reg alu_branch_cond;
    reg alu_jump;
    reg alu_set_compare;

    reg [31:0] rs1d_e, rs2d_e;
    reg [4:0] rs1_e, rs2_e, rd_e;
    reg [31:0] pc_e, immext_e;
    wire [31:0] pctarget_e;
    wire pcsrc_e;

    always @(posedge clk) begin
        if (reset || flushe)
            {alu_branch_cond, alu_jump} <= 0;
        else begin
            pc_e <= pc_d;

            // registers
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

            // alu opcodes and control
            alu_add  <= |{instr_lui, instr_auipc, instr_jal, instr_jalr, instr_addi, instr_add, is_lb_lh_lw_lbu_lhu, is_sb_sh_sw};
            alu_sub  <= instr_sub;
            alu_shl  <= instr_sll || instr_slli;
            alu_xor  <= instr_xor || instr_xori;
            alu_shrl <= instr_srli || instr_srl;
            alu_shra <= instr_srai || instr_sra;
            alu_or   <= instr_or || instr_ori;
            alu_and  <= instr_and || instr_andi;

            alu_eq   <= instr_beq;
            alu_neq  <= instr_bne;
            alu_lt   <= |{instr_slt, instr_slti, instr_blt};
            alu_ltu  <= |{instr_sltu, instr_sltiu, instr_bltu};
            alu_ge   <= instr_bge;
            alu_geu  <= instr_bgeu;

            alu_0_op1     <= instr_lui;
            alu_pc_op1    <= |{instr_auipc, instr_jal};
            alu_shamt_op2 <= is_slli_srli_srai;
            alu_imm_op2   <= !(|{is_slli_srli_srai, is_alu_reg_reg, is_beq_bne_blt_bge_bltu_bgeu});

            alu_branch_cond  <= is_beq_bne_blt_bge_bltu_bgeu;
            alu_jump <= instr_jal || instr_jalr;
            alu_set_compare  <= |{instr_slt, instr_sltu, instr_slti, instr_sltiu};
        end
    end

    // operands
    reg [31:0] alu_out;
    reg [31:0] a, b;
    always @* begin
        (* parallel_case *)
        case (1'b1)
            alu_0_op1: a = 0;
            alu_pc_op1: a = pc_e;
            default: a = src1;
        endcase

        (* parallel_case *)
        case (1'b1)
            alu_shamt_op2: b = rs2_e;
            alu_imm_op2: b = immext_e;
            default: b = src2;
        endcase
    end

    always @* begin
        (* parallel_case *)
        case(1'b1)
            alu_add:  alu_out = a + b;
            alu_sub:  alu_out = a - b;
            alu_shl:  alu_out = a << b[4:0];
            alu_xor:  alu_out = a ^ b;
            alu_shrl: alu_out = a >> b[4:0];
            alu_shra: alu_out = $signed(a) >>> b[4:0];
            alu_or:   alu_out = a | b;
            alu_and:  alu_out = a & b;
            default:  alu_out = 32'bx;
        endcase
    end

    // comparison ops
    reg alu_zero;
    wire eq = a == b;
    wire lts = $signed(a) < $signed(b);
    wire lt = a < b;
    always @* begin
        (* parallel_case *)
        case (1'b1)
            alu_eq:  alu_zero = eq;
            alu_neq: alu_zero = !eq;
            alu_lt:  alu_zero = lts;
            alu_ltu: alu_zero = lt;
            alu_ge:  alu_zero = !lts;
            alu_geu: alu_zero = !lt;
            default: alu_zero = 'bx;
        endcase
    end

    wire [31:0] alu_pcplus4 = pc_e + 4;
    wire [31:0] alu_pcplusimm = pc_e + immext_e;

    // TODO: redundant pc + imm in both alu and extra adder
    // extra adder required for conditional branches where alu is used to compare
    // should auipc, jal just use pc + imm adder instead of alu
    assign pctarget_e = alu_branch_cond ? alu_pcplusimm : alu_out;
    assign pcsrc_e = alu_branch_cond && alu_zero || alu_jump;

    reg [31:0] alu_result;
    (* parallel_case *)
    always @* case (1'b1)
        alu_jump: alu_result = alu_pcplus4;
        alu_set_compare: alu_result = alu_zero;
        default: alu_result = alu_out;
    endcase

    // execute to memory / writeback
    always @(posedge clk) begin
        rd_m <= rd_e;
        rd_w <= rd_m;

        wdata_m = src2;

        aluresult_m <= alu_result;
        aluresult_w <= aluresult_m;
    end

    // hazard logic
    wire [1:0] forward1, forward2;
    wire flushd, flushe, stallf, stalld;

    reg [31:0] src1, src2;
    always @* begin
        case(forward1)
           2'b00: src1 = rs1d_e;
           2'b01: src1 = result_w;
           2'b10: src1 = aluresult_m;
        endcase
        case(forward2)
           2'b00: src2 = rs2d_e;
           2'b01: src2 = result_w;
           2'b10: src2 = aluresult_m;
        endcase
    end

    hazard hzd (
        .regwrite_m(regwrite_m), .regwrite_w(regwrite_w),
        .rs1_e(rs1_e), .rs2_e(rs2_e), .rd_m(rd_m), .rd_w(rd_w),
        .rs1_d(instr_d[19:15]), .rs2_d(instr_d[24:20]), .rd_e(rd_e),
        .forward1(forward1), .forward2(forward2),
        .pcsrc_e(pcsrc_e), .flushd(flushd), .flushe(flushe),
        .stallf(stallf), .stalld(stalld), .is_load_e(is_load_e)
    );

    // writeback
    reg [4:0] rd_w;
    reg [31:0] aluresult_w;
    wire [31:0] result_w = is_load_w ? rdata_w : aluresult_w;
    always @(negedge clk)
        if (regwrite_w) rf[rd_w] <= result_w;
endmodule

module hazard (
    input regwrite_m, regwrite_w,
    input [4:0] rs1_d, rs2_d, rs1_e, rs2_e, rd_e, rd_m, rd_w,
    output reg [1:0] forward1, forward2,
    input is_load_e,
    input pcsrc_e,
    output stalld, stallf,
    output flushd, flushe
);
    // forward
    always @(*) begin
        if ((regwrite_m & rs1_e == rd_m) & rs1_e != 0)
            forward1 = 2'b10;
        else if ((regwrite_w & rs1_e == rd_w) & rs1_e != 0)
            forward1 = 2'b01;
        else
            forward1 = 2'b00;

        if ((regwrite_m & rs2_e == rd_m) & rs2_e != 0)
            forward2 = 2'b10;
        else if ((regwrite_w & rs2_e == rd_w) & rs2_e != 0)
            forward2 = 2'b01;
        else
            forward2 = 2'b00;
    end

    // stall
    wire load_stall = (rd_e == rs1_d | rd_e == rs2_d) & is_load_e;
    assign {stallf, stalld} = {2{load_stall}};

    // flush
    assign flushd = pcsrc_e;
    assign flushe = pcsrc_e | load_stall;
endmodule
