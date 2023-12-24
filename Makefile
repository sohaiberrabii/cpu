# icebreaker board
FREQ = 12
PIN_DEF = icebreaker.pcf
FAMILY = u
DEVICE = up5k
PACKAGE = sg48

CPU = pipelined

all: demo.rpt demo.bin

sim: cpu_tb.vcd

cpu_tb.vcd: cpu_tb.vvp
	vvp -N $< +vcd=$@

cpu_tb.vvp: cpu_tb.v $(CPU).v spram.v
	iverilog -g2005-sv -Wall -DSIM -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

demo.json: demo.v $(CPU).v spram.v uart.v
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
	rm -f demo.bin demo.json demo.rpt demo.asc cpu_tb.vcd cpu_syn.v cpu_syn_tb.vcd *.o *.elf *.hex

.PHONY: all prog sim synsim
.PRECIOUS: cpu_tb.vcd
