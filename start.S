.text

.global hello

start:

	addi x1, zero, 0
	addi x2, zero, 0
	addi x3, zero, 0
	addi x4, zero, 0
	addi x5, zero, 0
	addi x6, zero, 0
	addi x7, zero, 0
	addi x8, zero, 0
	addi x9, zero, 0
	addi x10, zero, 0
	addi x11, zero, 0
	addi x12, zero, 0
	addi x13, zero, 0
	addi x14, zero, 0
	addi x15, zero, 0
	addi x16, zero, 0
	addi x17, zero, 0
	addi x18, zero, 0
	addi x19, zero, 0
	addi x20, zero, 0
	addi x21, zero, 0
	addi x22, zero, 0
	addi x23, zero, 0
	addi x24, zero, 0
	addi x25, zero, 0
	addi x26, zero, 0
	addi x27, zero, 0
	addi x28, zero, 0
	addi x29, zero, 0
	addi x30, zero, 0
	addi x31, zero, 0

    jal hello

// test infra from picorv32 and riscv-tests
# define TEST(n) \
	.global n; \
	jal zero,n; \
	.global n ## _ret; \
	n ## _ret:

	TEST(lui)
	TEST(auipc)
	TEST(j)
	TEST(jal)
	TEST(jalr)

	TEST(beq)
	TEST(bne)
	TEST(blt)
	TEST(bge)
	TEST(bltu)
	TEST(bgeu)

	TEST(lb)
	TEST(lh)
	TEST(lw)
	TEST(lbu)
	TEST(lhu)

	TEST(sb)
	TEST(sh)
	TEST(sw)

	TEST(addi)
	TEST(slti)
	TEST(xori)
	TEST(ori)
	TEST(andi)
	TEST(slli)
	TEST(srli)
	TEST(srai)

	TEST(add)
	TEST(sub)
	TEST(sll)
	TEST(slt)
	TEST(xor)
	TEST(srl)
	TEST(sra)
	TEST(or)
	TEST(and)

    // Get cycle and instruction count
    li x2, 0x2000c
cycle_count:
    rdcycleh x3
    rdcycle x4
    rdcycleh x5
    bne x3, x5, cycle_count
    sw x4, 0(x2)
    sw x5, 4(x2)

instr_count:
    rdinstreth x3
    rdinstret x4
    rdinstreth x5
    bne x3, x5, instr_count
    sw x4, 8(x2)
    sw x5, 12(x2)

    li x2, 0x20008
    li x1, 0
    sw x1, 0(x2)
