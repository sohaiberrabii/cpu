#define TESTNUM x30

#define TEST_CASE(testnum, rd, expected, instrs...) \
    instrs; \
    li x31, expected; \
    li TESTNUM, testnum; \
    bne rd, x31, fail

#define TEST_RR(testnum, inst, expected, val1, val2) \
    TEST_CASE(testnum, x3, expected, \
      li  x1, val1; \
      li  x2, val2; \
      inst x3, x1, x2; \
    )

#define TEST_BR_TAKEN(testnum, inst, val1, val2 ) \
    li  TESTNUM, testnum; \
    li  x1, val1; \
    li  x2, val2; \
    inst x1, x2, 2f; \
    bne x0, TESTNUM, fail; \
1:  bne x0, TESTNUM, 3f; \
2:  inst x1, x2, 1b; \
    bne x0, TESTNUM, fail; \
3:

