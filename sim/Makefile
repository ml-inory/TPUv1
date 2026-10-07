# =============================================================================
#  TPUv1 - simulation build (PE)
#
#  Simulator : Icarus Verilog (iverilog + vvp)
#  Usage     : cd sim && make [target]
#
#  Targets:
#    all / run   compile + run the testbench (default)
#    build       compile only (produces build/pe_tb.vvp)
#    wave        run and open the VCD in gtkwave
#    lint        parse/elaborate only, show warnings
#    clean       remove all build artifacts
#    help        show this message
#
#  Overridable variables (e.g. `make TOP=foo_tb`):
#    TOP       top-level testbench module / file stem   (default: pe_tb)
#    RTL_DIR   directory holding synthesizable sources  (default: ../rtl)
#    TB_DIR    directory holding testbench sources       (default: ../tb)
#    IVFLAGS   extra iverilog flags
# =============================================================================

# ---- Tools ------------------------------------------------------------------
IV      ?= iverilog
VVP     ?= vvp
WAVE    ?= gtkwave

# ---- Design / paths ---------------------------------------------------------
TOP       ?= pe_tb
RTL_DIR   ?= ../rtl
TB_DIR    ?= ../tb
BUILD_DIR ?= build

SIM_BIN   := $(BUILD_DIR)/$(TOP).vvp
WAVE_FILE := $(BUILD_DIR)/$(TOP).vcd

# ---- Compile flags ----------------------------------------------------------
IVFLAGS ?= -g2012 -Wall -Wno-timescale
IVFLAGS += -s $(TOP)                      # force testbench as the root module

# ---- Source discovery -------------------------------------------------------
RTL_SRCS := $(wildcard $(RTL_DIR)/*.sv) $(wildcard $(RTL_DIR)/*.v)
TB_SRCS  := $(wildcard $(TB_DIR)/*.sv)  $(wildcard $(TB_DIR)/*.v)
SRCS     := $(RTL_SRCS) $(TB_SRCS)

# =============================================================================
.PHONY: all run build compile wave lint clean help

all: run

# ---- Compile ----------------------------------------------------------------
build compile: $(SIM_BIN)

$(SIM_BIN): $(SRCS)
	@mkdir -p $(BUILD_DIR)
	$(IV) $(IVFLAGS) -o $@ $(SRCS)
	@echo ">> built $(SIM_BIN)"

# ---- Run --------------------------------------------------------------------
run: $(SIM_BIN)
	$(VVP) $(SIM_BIN) +wave=$(WAVE_FILE)

# ---- Waveform ---------------------------------------------------------------
wave: $(SIM_BIN)
	$(VVP) $(SIM_BIN) +wave=$(WAVE_FILE)
	$(WAVE) $(WAVE_FILE) &

# ---- Lint (parse + elaborate, no run) ---------------------------------------
lint:
	$(IV) $(IVFLAGS) -t null $(SRCS)
	@echo ">> lint OK"

# ---- Clean ------------------------------------------------------------------
clean:
	rm -rf $(BUILD_DIR)

# ---- Help -------------------------------------------------------------------
help:
	@echo "TPUv1 PE simulation"
	@echo ""
	@echo "  make            compile and run the testbench"
	@echo "  make build      compile only -> $(SIM_BIN)"
	@echo "  make wave       run and open $(WAVE_FILE) in gtkwave"
	@echo "  make lint       elaborate only, report warnings"
	@echo "  make clean      remove $(BUILD_DIR)/"
	@echo ""
	@echo "  variables: TOP=$(TOP) RTL_DIR=$(RTL_DIR) TB_DIR=$(TB_DIR)"
	@echo "  sources  : $(SRCS)"
