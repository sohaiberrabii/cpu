# icebreaker board
FREQ = 12
PIN_DEF = icebreaker.pcf
FAMILY = u
DEVICE = up5k
PACKAGE = sg48

# riscv prefix
TOOLCHAIN_PREFIX = riscv32-unknown-elf-

TEST_OBJS = $(addsuffix .o,$(basename $(wildcard test/*.S)))

all: icebreaker.rpt icebreaker.bin firmware.hex

sim: cpu_tb.vcd

synsim: cpu_syntb.vcd

%.vcd: %.vvp firmware.hex
	vvp -N $< +vcd=$@

cpu_syntb.vcd: cpu_syntb.vvp firmware.hex
	vvp -N $< +vcd=$@

cpu_tb.vvp: cpu_tb.v cpu.v
	iverilog -g2005-sv -o $@ $^

cpu_syntb.vvp: cpu_tb.v synth.v
	iverilog -o $@ $^

synth.v: cpu.v
	yosys -qv3 -l synth.log -p 'read_verilog $<; hierarchy -top cpu; synth; write_verilog $@'

ibsynth.v: cpu.v
	yosys -p 'synth_ice40 -device $(FAMILY) -top cpu; write_verilog $@' $^

icebsim: icebreaker_tb.vcd

icebreaker_tb.vvp: icebreaker_tb.v icebreaker.v spram.v uart.v cpu.v
	iverilog -g2005-sv -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

icebreaker.json: icebreaker.v spram.v uart.v cpu.v
	yosys -p 'synth_ice40 -device $(FAMILY) -top icebreaker -json $@' $^

icebreaker.asc: icebreaker.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --asc $@ --pcf $(PIN_DEF) --json $<

icebreaker.bin: icebreaker.asc
	icepack -s $< $@

icebreaker.rpt: icebreaker.asc
	icetime -d up5k -c 12 -mtr $@ $<

prog: icebreaker.bin
	iceprog $<

test/%.o: test/%.S test/riscv_test.h test/test_macros.h
	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32im -o $@ \
		-DTEST_FUNC_NAME=$(notdir $(basename $<)) \
		-DTEST_FUNC_TXT='"$(notdir $(basename $<))"'\
		-DTEST_FUNC_RET=$(notdir $(basename $<))_ret $<

%.o: %.S
	$(TOOLCHAIN_PREFIX)gcc -c -mabi=ilp32 -march=rv32i  -o $@ $<

firmware.bin: firmware.elf
	$(TOOLCHAIN_PREFIX)objcopy -O binary $< $@

firmware.elf: start.o $(TEST_OBJS) sections.lds
	$(TOOLCHAIN_PREFIX)ld -o $@ -T sections.lds start.o $(TEST_OBJS)

firmware.hex: firmware.bin makehex.py
	python3 makehex.py $< 32768 > $@

dis: firmware.elf
	riscv32-unknown-elf-objdump -Mnumeric -d $< > firmware.dis

clean:
	rm -f icebreaker.{json,rpt,asc} *.log *.vvp *.vcd synth.v *.o *.elf *.hex *.bin *.dis

.PHONY: all prog sim synsim icebsim dis
.PRECIOUS: cpu_tb.vcd
