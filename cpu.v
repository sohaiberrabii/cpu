`default_nettype none

module cpu(
    input clk, reset,
    output [31:0] mem_addr,
    output reg [31:0] mem_wdata,
    output [3:0]  mem_write,
    input  [31:0] mem_rdata,
    input  [31:0] instr,
    output [31:0] pc
);
    // register file
    reg [31:0] rf[31:0];

    // cycle and retired instr counters
    reg [63:0] cycle_counter, instr_counter;
    always @(posedge clk) begin
        if (reset) begin
           cycle_counter <= 0;
           instr_counter <= 0;
        end else begin
            cycle_counter <= cycle_counter + 1;
            // an instruction is guaranteed to complete if not flushed at execute stage
            if (!flushe)
                instr_counter <= instr_counter + 1;
        end
    end

    assign mem_addr  = alu_result;
    assign mem_write = memwrite_e;
    assign pc        = pc_f;

    // memory stage signals
    reg [4:0] rd_m;
    reg [31:0] aluresult_m;

    // fetch
    wire [31:0] instr_f = instr;
    reg [31:0] pc_f;
    always @(posedge clk) begin
        if (reset)
            pc_f <= 0;
        else if (~load_stall)
            if (take_branch)
                pc_f <= {pctarget_e[31:1], 1'b0};
            else
                pc_f <= pc_f + 4;
    end

    // decode
    reg [31:0] instr_d, pc_d;

    // TODO: given synchronous imem can these extra ffs be removed?
    always @(posedge clk) begin
        if (reset | flushd)
            {instr_d, pc_d} <= 0;
        else if (~load_stall)
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

    // csr
    wire csr = instr_d[6:0] == 7'b1110011;
    wire instr_rdcycle  = csr && instr_d[31:12] == {12'hc00, 5'b0, 3'b010};
    wire instr_rdcycleh = csr && instr_d[31:12] == {12'hc80, 5'b0, 3'b010};
    wire instr_rdinstret  = csr && instr_d[31:12] == {12'hc02, 5'b0, 3'b010};
    wire instr_rdinstreth = csr && instr_d[31:12] == {12'hc82, 5'b0, 3'b010};
    wire is_rdcycle_rdinstret = |{instr_rdcycle, instr_rdcycleh, instr_rdinstret, instr_rdinstreth};

    // control lines for load/store
    reg [3:0] memwrite_e;
    reg sb_e, sh_e, sw_e;

    reg regwrite_e, regwrite_m, regwrite_w;

    reg is_load_e;
    reg lw_e, lw_m, lw_w;
    reg lh_e, lh_m, lh_w;
    reg lhu_e, lhu_m, lhu_w;
    reg lb_e, lb_m, lb_w;
    reg lbu_e, lbu_m, lbu_w;

    reg rdcycleh_e, rdcycle_e;
    reg rdinstret_e, rdinstreth_e;

    always @(posedge clk) begin
        if (reset || flushe) begin
            regwrite_e <= 0;
            is_load_e <= 0;
            {sb_e, sh_e, sw_e} <= 0;
        end else begin
            regwrite_e <= |{
                is_lui_auipc_jal_jalr,
                is_lb_lh_lw_lbu_lhu,
                is_alu_reg_imm,
                is_alu_reg_reg,
                is_rdcycle_rdinstret
            };

            is_load_e <= is_lb_lh_lw_lbu_lhu;
            lw_e <= instr_lw;
            lh_e <= instr_lh;
            lhu_e <= instr_lhu;
            lb_e <= instr_lb;
            lbu_e <= instr_lbu;

            sb_e <= instr_sb;
            sh_e <= instr_sh;
            sw_e <= instr_sw;

            rdcycle_e <= instr_rdcycle;
            rdcycleh_e <= instr_rdcycleh;
            rdinstret_e <= instr_rdinstret;
            rdinstreth_e <= instr_rdinstreth;
        end

        regwrite_m <= regwrite_e;
        regwrite_w <= regwrite_m;

        lw_m <= lw_e;
        lw_w <= lw_m;

        lh_m <= lh_e;
        lh_w <= lh_m;

        lhu_m <= lhu_e;
        lhu_w <= lhu_m;

        lb_m <= lb_e;
        lb_w <= lb_m;

        lbu_m <= lbu_e;
        lbu_w <= lbu_m;
    end

    (* parallel_case *)
    always @* case(1'b1)
        sb_e: begin
            memwrite_e = 4'b0001 << alu_result[1:0];
            mem_wdata = {4{src2[7:0]}};
        end
        sh_e: begin
            memwrite_e = alu_result[1] ? 4'b1100 : 4'b0011;
            mem_wdata = {2{src2[15:0]}};
        end
        sw_e: begin
            memwrite_e = 4'b1111;
            mem_wdata = src2;
        end
        default: begin
            memwrite_e = 4'b0;
            mem_wdata = 32'bx;
        end
    endcase

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
    wire take_branch;

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
    assign take_branch = alu_branch_cond && alu_zero || alu_jump;

    reg [31:0] alu_result;
    (* parallel_case *)
    always @* case (1'b1)
        alu_jump:        alu_result = alu_pcplus4;
        alu_set_compare: alu_result = alu_zero;
        rdcycle_e:       alu_result = cycle_counter[31:0];
        rdcycleh_e:      alu_result = cycle_counter[63:32];
        rdinstret_e:     alu_result = instr_counter[31:0];
        rdinstreth_e:    alu_result = instr_counter[63:32];
        default:         alu_result = alu_out;
    endcase

    // execute to memory / writeback
    always @(posedge clk) begin
        rd_m <= rd_e;
        rd_w <= rd_m;

        aluresult_m <= alu_result;
        aluresult_w <= aluresult_m;
    end

    // stall
    wire load_stall = (rd_e == instr_d[19:15] | rd_e == instr_d[24:20]) & is_load_e;

    // flush
    wire flushd = take_branch;
    wire flushe = take_branch | load_stall;

    // forward
    reg [31:0] src1, src2;
    always @* begin
        if ((regwrite_m & rs1_e == rd_m) & rs1_e != 0)
            src1 = aluresult_m;
        else if ((regwrite_w & rs1_e == rd_w) & rs1_e != 0)
            src1 = result_w;
        else
            src1 = rs1d_e;

        if ((regwrite_m & rs2_e == rd_m) & rs2_e != 0)
            src2 = aluresult_m;
        else if ((regwrite_w & rs2_e == rd_w) & rs2_e != 0)
            src2 = result_w;
        else
            src2 = rs2d_e;
    end

    // writeback
    reg [4:0] rd_w;
    reg [31:0] aluresult_w;
    reg [31:0] result_w;
    reg [31:0] rdata;

    (* parallel_case *)
    always @(posedge clk) case(1'b1)
        lw_m: rdata <= mem_rdata;
        lhu_m || lh_m: case (aluresult_m[1])
            1'b0: rdata <= {16'b0, mem_rdata[15:0]};
            1'b1: rdata <= {16'b0, mem_rdata[31:16]};
        endcase
        lbu_m || lb_m: case (aluresult_m[1:0])
            2'b00: rdata <= {24'b0, mem_rdata[7:0]};
            2'b01: rdata <= {24'b0, mem_rdata[15:8]};
            2'b10: rdata <= {24'b0, mem_rdata[23:16]};
            2'b11: rdata <= {24'b0, mem_rdata[31:24]};
        endcase
        default: rdata <= 32'bx;
    endcase

    (* parallel_case *)
    always @* case(1'b1)
        |{lw_w, lhu_w, lbu_w}: result_w = rdata;
        lh_w: result_w = $signed(rdata[15:0]);
        lb_w: result_w = $signed(rdata[7:0]);
        default: result_w = aluresult_w;
    endcase

    always @(negedge clk)
        if (regwrite_w) rf[rd_w] <= result_w;
endmodule
