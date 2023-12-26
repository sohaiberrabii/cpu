#include "test_helpers.h"

.text
start:
    // test isa
    // addi: init register
    TEST_CASE(1, x1, 0x00000001, addi x1, zero, 1)
    TEST_CASE(2, x1, 0x00000003, addi x1, zero, 3)

    // lui
    TEST_CASE(3, x1, 0xabcde000, lui x1, 0xabcde)
    TEST_CASE(4, x0, 0, lui x0, 0x10000)

    // add
    TEST_RR(5,  add, 0x00000000, 0x00000000, 0x00000000)
    TEST_RR(6,  add, 0x00000002, 0x00000001, 0x00000001)
    TEST_RR(7,  add, 0x0000000a, 0x00000003, 0x00000007)
    TEST_RR(8,  add, 0x0abcde00, 0x80000000, 0x8abcde00)

    li x2, 0x20008
    sw zero, 0(x2)

// infinite loop
pass: beq x2, x2, pass

fail:
    li x2, 0x20008
    sw x30, 0(x2)
