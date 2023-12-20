# icebreaker board
FREQ = 12
PIN_DEF = icebreaker.pcf
FAMILY = u
DEVICE = up5k
PACKAGE = sg48

CPU = pipelined

# TEST_OBJS = $(addsuffix .o,$(basename $(wildcard tests/*.S)))
# FIRMWARE_OBJS = firmware/start.o firmware/irq.o firmware/print.o firmware/hello.o firmware/sieve.o firmware/multest.o firmware/stats.o
# RISCV_GNU_TOOLCHAIN_INSTALL_PREFIX = /opt/riscv32
# TOOLCHAIN_PREFIX = $(RISCV_GNU_TOOLCHAIN_INSTALL_PREFIX)i/bin/riscv32-unknown-elf-

all: demo.rpt demo.bin

sim: cpu_tb.vcd

# firmware/firmware.hex: firmware/firmware.bin firmware/makehex.py
# 	python3 firmware/makehex.py $< 32768 > $@

# firmware/firmware.bin: firmware/firmware.elf
# 	$(TOOLCHAIN_PREFIX)objcopy -O binary $< $@
# 	chmod -x $@

# firmware/firmware.elf: $(FIRMWARE_OBJS) $(TEST_OBJS) firmware/sections.lds
# 	$(TOOLCHAIN_PREFIX)gcc -Os -mabi=ilp32 -march=rv32im$(subst C,c,$(COMPRESSED_ISA)) -ffreestanding -nostdlib -o $@ \
# 		-Wl,--build-id=none,-Bstatic,-T,firmware/sections.lds,-Map,firmware/firmware.map,--strip-debug \
# 		$(FIRMWARE_OBJS) $(TEST_OBJS) -lgcc
# 	chmod -x $@

# firmware/start.o: firmware/start.S
# 	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32im$(subst C,c,$(COMPRESSED_ISA)) -o $@ $<

# firmware/%.o: firmware/%.c
# 	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32i$(subst C,c,$(COMPRESSED_ISA)) -Os --std=c99 $(GCC_WARNS) -ffreestanding -nostdlib -o $@ $<

# tests/%.o: tests/%.S tests/riscv_test.h tests/test_macros.h
# 	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32im -o $@ -DTEST_FUNC_NAME=$(notdir $(basename $<)) \
# 		-DTEST_FUNC_TXT='"$(notdir $(basename $<))"' -DTEST_FUNC_RET=$(notdir $(basename $<))_ret $<

cpu_tb.vcd: cpu_tb.vvp
	vvp -N $< +vcd=$@

cpu_tb.vvp: cpu_tb.v $(CPU).v spram.v
	iverilog -g2005-sv -Wall -DSIM -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

demo.json: demo.v $(CPU).v spram.v
	yosys -p 'synth_ice40 -device $(FAMILY) -top top -json $@' $^

demo.asc: demo.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --asc $@ --pcf $(PIN_DEF) --json $<

demo.bin: demo.asc
	icepack -s $< $@

demo.rpt: demo.asc
	icetime -d up5k -c 12 -mtr $@ $<

prog: demo.bin
	iceprog $<

clean:
	rm -f demo.bin demo.json demo.rpt demo.asc cpu_tb.vcd cpu_syn.v cpu_syn_tb.vcd

.PHONY: all prog sim synsim
.PRECIOUS: cpu_tb.vcd
