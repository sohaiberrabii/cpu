#include "test_helpers.h"

.text
start:
    // Tests from https://github.com/YosysHQ/picorv32

    // addi: init register
    TEST_CASE(1, x1, 0x00000001, addi x1, zero, 1)
    TEST_CASE(2, x1, 0x00000003, addi x1, zero, 3)

    // lui
    TEST_CASE(3, x1, 0xabcde000, lui x1, 0xabcde)
    TEST_CASE(4, x0, 0, lui x0, 0x10000)

    /* // auipc */
    TEST_CASE(5, a0, 10000,
      lla a0, 1f + 10000; \
      jal a1, 1f; \
      1: sub a0, a0, a1
    )
    TEST_CASE(6, a0, -10000, \
      lla a0, 1f - 10000; \
      jal a1, 1f; \
      1: sub a0, a0, a1
    )

    // add
    TEST_RR(7,  add, 0x00000000, 0x00000000, 0x00000000)
    TEST_RR(8,  add, 0x00000000, 0x00000001, 0xffffffff)
    TEST_RR(9,  add, 0x0000000a, 0x00000003, 0x00000007)
    TEST_RR(10, add, 0x0abcde00, 0x80000000, 0x8abcde00)

    // jump tests
    // ====================

    // j
    li TESTNUM, 11
    j test_next
    j fail
test_next:
    TEST_CASE(12, x1, 3, \
        li  x1, 1; \
        j 1f; \
        addi x1, x1, 1; \
        addi x1, x1, 1; \
    1:  addi x1, x1, 1; \
        addi x1, x1, 1; \
    )

    // jal
    li  TESTNUM, 13
    li  ra, 0
    linkaddr_2:
      jal target_2
      nop
      nop
      j fail
    target_2:
      la  x2, linkaddr_2
      addi x2, x2, 4
      bne x2, ra, fail
    TEST_CASE(14, x2, 3, \
        li  x2, 1; \
        jal 1f; \
        addi x2, x2, 1; \
        addi x2, x2, 1; \
    1:  addi x2, x2, 1; \
        addi x2, x2, 1; \
    )

    // jalr
    li  TESTNUM, 15
    li  x31, 0
    la  x2, target_3
    linkaddr_3:
      jalr x19, x2, 0
      nop
      nop
      j fail
    target_3:
      la  x1, linkaddr_3
      addi x1, x1, 4
      bne x1, x19, fail

    li  TESTNUM, 16
    li  x31, 0
    la  x3, target_4
    linkaddr_4:
      jalr x0, x3, 0
      nop
      j fail
    target_4:
      bne x31, x0, fail

    // taken branch tests
    // ===================

    // beq
    TEST_BR_TAKEN(17, beq,  0,  0);
    TEST_BR_TAKEN(18, beq,  1,  1);
    TEST_BR_TAKEN(19, beq, -1, -1);

    // bne
    TEST_BR_TAKEN(17, bne,  0,  1);
    TEST_BR_TAKEN(18, bne,  1,  0);
    TEST_BR_TAKEN(19, bne, -1,  1);
    TEST_BR_TAKEN(20, bne,  1, -1);

    // bge
    TEST_BR_TAKEN(21, bge,  0,  0);
    TEST_BR_TAKEN(22, bge,  1,  1);
    TEST_BR_TAKEN(23, bge, -1, -1);
    TEST_BR_TAKEN(24, bge,  1,  0);
    TEST_BR_TAKEN(25, bge,  1, -1);
    TEST_BR_TAKEN(26, bge, -1, -2);

    // tests passed
    li x2, 0x20004
    sw zero, 0(x2)
// infinite loop
pass: beq x2, x2, pass

fail:
    li x2, 0x20004
    sw x30, 0(x2)
