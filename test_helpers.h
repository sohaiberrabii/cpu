#define TEST_CASE(testnum, rd, expected, instrs...) \
    instrs; \
    li x31, expected; \
    li x30, testnum; \
    bne rd, x31, fail

#define TEST_RR(testnum, inst, expected, val1, val2) \
    TEST_CASE(testnum, x3, expected, \
      li  x1, val1; \
      li  x2, val2; \
      inst x3, x1, x2; \
    )
