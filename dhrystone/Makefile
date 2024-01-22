OBJS = dhry_1.o dhry_2.o stdlib.o start.o
CFLAGS = -MD -O3 -mabi=ilp32 -march=rv32i -DTIME -DRISCV -DUSE_MYSTDLIB -ffreestanding -nostdlib
TOOLCHAIN_PREFIX = riscv32-unknown-elf-

test: testbench.vvp dhry.hex
	vvp -N testbench.vvp

testbench.vvp: testbench.v ../cpu.v
	iverilog -o testbench.vvp testbench.v ../cpu.v
	chmod -x testbench.vvp

dhry.hex: dhry.bin ../makehex.py
	python3 ../makehex.py $< 65536 > $@

dhry.bin: dhry.elf
	$(TOOLCHAIN_PREFIX)objcopy -O binary $< $@

dhry.elf: $(OBJS) sections.lds
	$(TOOLCHAIN_PREFIX)gcc $(CFLAGS) -Wl,-Bstatic,-T,sections.lds,-Map,dhry.map,--strip-debug -o $@ $(OBJS) -lgcc
	chmod -x $@

%.o: %.c
	$(TOOLCHAIN_PREFIX)gcc -c $(CFLAGS) $<

%.o: %.S
	$(TOOLCHAIN_PREFIX)gcc -c $(CFLAGS) $<

dhry_1.o dhry_2.o: CFLAGS += -Wno-implicit-int -Wno-implicit-function-declaration

clean:
	rm -rf *.o *.d dhry.elf dhry.map dhry.bin dhry.hex testbench.vvp testbench.vcd

.PHONY: test clean

-include *.d

