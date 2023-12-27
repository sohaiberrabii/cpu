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

    // add
    TEST_RR(5,  add, 0x00000000, 0x00000000, 0x00000000)
    TEST_RR(6,  add, 0x00000002, 0x00000001, 0x00000001)
    TEST_RR(7,  add, 0x0000000a, 0x00000003, 0x00000007)
    TEST_RR(8,  add, 0x0abcde00, 0x80000000, 0x8abcde00)

    // jump tests
    li TESTNUM, 9
    j test_next
    j fail
test_next:

    // delay slots
    TEST_CASE(10, x2, 3, \
        li  x2, 1; \
        jal 1f; \
        addi x2, x2, 1; \
        addi x2, x2, 1; \
    1:  addi x2, x2, 1; \
        addi x2, x2, 1; \
    )
    TEST_CASE(10, x1, 3, \
        li  x1, 1; \
        j 1f; \
        addi x1, x1, 1; \
        addi x1, x1, 1; \
    1:  addi x1, x1, 1; \
        addi x1, x1, 1; \
    )

    li x2, 0x20004
    sw zero, 0(x2)

// infinite loop
pass: beq x2, x2, pass

fail:
    li x2, 0x20004
    sw x30, 0(x2)
