PROJ = cpu

# alchitry cu
FREQ = 100 # without PLL, alchitry cu runs at 100MHz
PIN_DEF = alchitrycu.pcf
FAMILY = hx
DEVICE = hx8k
PACKAGE = cb132

# icebreaker
#FREQ = 12
#PIN_DEF = icebreaker.pcf
#FAMILY = u
#DEVICE = up5k
#PACKAGE = sg48

ifneq ($(wildcard Makefile.conf),)
include Makefile.conf
endif

all: $(PROJ).bin

%_single_tb: %_tb.v %.v
	iverilog -g2005 -Wall -DSIM -DSINGLE -o $@ $^

%_pipelined_tb: %_tb.v %.v
	iverilog -g2005 -Wall -DSIM -DPIPELINED -o $@ $^

%_tb.vcd: %_tb
	vvp -N $< +vcd=$@

sim-single: $(PROJ)_single_tb.vcd
sim-pipelined: $(PROJ)_pipelined_tb.vcd

syn: $(PROJ)_syn.v

%_syn.v: %.json
	yosys -p 'read_json $^; write_verilog $@; show'

%_pp.v: %.v
	iverilog -E -o $@ $(DEFALCHITRY) $<

%.json: %_pp.v
	yosys -p 'synth_ice40 -device $(FAMILY) -top $(PROJ) -json $@' $<

%.asc: $(PIN_DEF) %.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --asc $@ --pcf $< --json $*.json

%.bin: %.asc
	icepack $< $@

prog: $(PROJ).bin
	iceprog $<

config-clean:
	rm -f Makefile.conf

config-icebreaker: clean
	echo 'FREQ = 12' > Makefile.conf
	echo 'PIN_DEF = icebreaker.pcf' >> Makefile.conf
	echo 'FAMILY = u' >> Makefile.conf
	echo 'DEVICE = up5k' >> Makefile.conf
	echo 'PACKAGE = sg48' >> Makefile.conf

config-alchitrycu: clean
	echo 'FREQ = 100' > Makefile.conf
	echo 'PIN_DEF = alchitrycu.pcf' >> Makefile.conf
	echo 'FAMILY = hx' >> Makefile.conf
	echo 'DEVICE = hx8k' >> Makefile.conf
	echo 'PACKAGE = cb132' >> Makefile.conf
	echo 'DEFALCHITRY = -DALCHITRYCU' >> Makefile.conf

clean:
	rm -f $(PROJ)_pp.v $(PROJ).json $(PROJ).asc $(PROJ).bin $(PROJ)_tb $(PROJ)_{single,pipelined}_tb.vcd $(PROJ)_syn.v

.PHONY: all prog clean config-clean config-icebreaker config-alchitrycu syn sim
.PRECIOUS: $(PROJ)_single_tb.vcd $(PROJ)_pipelined_tb.vcd
