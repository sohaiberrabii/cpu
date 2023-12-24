# icebreaker board
FREQ = 12
PIN_DEF = icebreaker.pcf
FAMILY = u
DEVICE = up5k
PACKAGE = sg48

all: icebreaker.rpt icebreaker.bin

sim: cpu_tb.vcd

cpu_tb.vcd: cpu_tb.vvp
	vvp -N $< +vcd=$@

cpu_tb.vvp: cpu_tb.v cpu.v spram.v
	iverilog -g2005-sv -Wall -DSIM -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

icebreaker.json: icebreaker.v cpu.v spram.v uart.v
	yosys -p 'synth_ice40 -device $(FAMILY) -top top -json $@' $^

icebreaker.asc: icebreaker.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --asc $@ --pcf $(PIN_DEF) --json $<

icebreaker.bin: icebreaker.asc
	icepack -s $< $@

icebreaker.rpt: icebreaker.asc
	icetime -d up5k -c 12 -mtr $@ $<

prog: icebreaker.bin
	iceprog $<

clean:
	rm -f icebreaker.{bin,json,rpt,asc} cpu_tb.vcd cpu_syn.v *.o *.elf *.hex

.PHONY: all prog sim
.PRECIOUS: cpu_tb.vcd
