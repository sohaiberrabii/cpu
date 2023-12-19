PROJ = cpu

# icebreaker board
FREQ = 12
PIN_DEF = icebreaker.pcf
FAMILY = u
DEVICE = up5k
PACKAGE = sg48

all: $(PROJ).rpt $(PROJ).bin

sim: $(PROJ)_tb.vcd

syn: $(PROJ)_syn.v

%_tb: %_tb.v %.v
	iverilog -g2001 -Wall -DSIM -o $@ $^

%_tb.vcd: %_tb
	vvp -N $< +vcd=$@

%_syn.v: %.json
	yosys -p 'read_json $^; write_verilog $@; show'

%_pp.v: %.v
	iverilog -E -o $@ $<

%.json: %_pp.v
	yosys -p 'synth_ice40 -device $(FAMILY) -top top -json $@' $<

%.asc: $(PIN_DEF) %.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --asc $@ --pcf $< --json $*.json

%.bin: %.asc
	icepack $< $@

%.rpt: %.asc
	icetime -d up5k -c 12 -mtr $@ $<

prog: $(PROJ).bin
	iceprog $<

clean:
	rm -f $(PROJ)_pp.v $(PROJ).json $(PROJ).asc $(PROJ).bin $(PROJ)_tb $(PROJ)_tb.vcd $(PROJ)_syn.v

.PHONY: all prog syn sim
.PRECIOUS: $(PROJ)_tb.vcd
