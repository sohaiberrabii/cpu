module single (
    input clk, reset,
    output mem_write,
    output [31:0] mem_addr,
    output [31:0] mem_wdata,
    input [31:0] instr, mem_rdata,
    output [31:0] pc
);
    wire branch, jump, zero, regwrite, alusrc;
    wire [1:0] resultsrc, immsrc;
    wire [2:0] aluctl;
    wire pcsrc = branch & zero | jump;

    controller ctl (
        .opcode(instr[6:0]), .funct3(instr[14:12]), .funct75(instr[30]),
        .alusrc(alusrc), .immsrc(immsrc), .resultsrc(resultsrc),
        .branch(branch), .jump(jump), .memwrite(memwrite), .regwrite(regwrite),
        .aluctl(aluctl)
    );

    datapath dp (
        .clk(clk), .reset(reset),
        .instr(instr), .rdata(mem_rdata),
        .pcsrc(pcsrc), .alusrc(alusrc), .resultsrc(resultsrc), .regwrite(regwrite),
        .immsrc(immsrc), .aluctl(aluctl),
        .pc(pc), .alures(mem_addr), .wdata(mem_wdata), .zero(zero)
    );
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

module datapath (
    input clk, reset,
    input [31:0] instr, rdata,
    input pcsrc, alusrc, regwrite,
    input [1:0] resultsrc, immsrc,
    input [2:0] aluctl,
    output zero,
    output [31:0] alures, wdata,
    output reg [31:0] pc
);
    wire [31:0] rs1d, rs2d, immext;
    reg [31:0] result;
    wire [31:0] pcplus4 = pc + 4;
    wire [31:0] pcplusimm = pc + immext;

    always @(posedge clk, posedge reset)
        if (reset)
            pc <= 0;
        else
            pc <= pcsrc ? pcplusimm : pcplus4;

    always @(*) case(resultsrc)
        2'b00: result = alures;
        2'b01: result = rdata;
        2'b10: result = pcplus4;
        default: result = 31'bx;
    endcase

    register_file reg_file (
        .clk(clk), .we3(regwrite),
        .addr1(instr[19:15]), .addr2(instr[24:20]), .addr3(instr[11:7]),
        .wd3(result),
        .rd1(rs1d), .rd2(wdata)
    );

    extend ext (.instr(instr[31:7]), .immsrc(immsrc), .immext(immext));

    alu alu (.a(rs1d), .b(alusrc ? immext : wdata), .ctl(aluctl), .res(alures), .zero(zero));
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

// TODO: experiment how different alu code is synthesized by yosys for eg:
// res = a + b;
// res = a - b;
// res = a & b;
// res = a | b;
// res = a < b;
module alu (input [31:0] a, b, input [2:0] ctl, output reg [31:0] res, output zero);
    wire [31:0] condnotb = ctl[0] ? ~b : b;
    wire [31:0] sum = a + condnotb + ctl[0]; // 2s comp

    // TODO: check if abc can optimize this
    // isadd: 000, issub: 001 or 101
    wire isadd = ~ctl[0] & ~ctl[1] & ~ctl[2];
    wire issub = ctl[0] & ~ctl[1];
    wire ovf = ~(a[31] ^ b[31]) & (a[31] ^ sum[31]) & isadd |
                (a[31] ^ b[31]) & (a[31] ^ sum[31]) & issub;

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
`ifdef SINGLE
    always @(posedge clk)
`else
    always @(negedge clk)
`endif
        if (we3) rf[addr3] <= wd3;

    assign rd1 = (addr1 != 0) ? rf[addr1] : 0;
    assign rd2 = (addr2 != 0) ? rf[addr2] : 0;
endmodule

module instruction_memory (input [31:0] addr, output [31:0] instr);
    reg [31:0] mem[0:255];
    initial $readmemh("riscvtest.txt", mem);
    assign instr = mem[addr[31:2]];
endmodule

module data_memory (
    input clk, we,
    input [31:0] addr, wdata,
    output [31:0] rdata
);
    reg [31:0] mem [255:0];
    always @(posedge clk)
        if (we)
            mem[addr[31:2]] <= wdata;
    assign rdata = mem[addr[31:2]];
endmodule
