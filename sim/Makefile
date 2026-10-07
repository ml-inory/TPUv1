# =============================================================================
#  TPUv1 - simulation build
#
#  Simulator : Icarus Verilog (iverilog + vvp)
#  Usage     : cd sim && make [<testbench>] [target]
#
#  Selecting a testbench
#    make                build + run $(TOP)             (default: pe_tb)
#    make TOP=<name>     same, explicit
#    make <name>         build + run an auto-discovered testbench
#    make list           list discovered testbenches
#    make regress        run every discovered testbench
#
#  Targets
#    all / run   compile + run                    (default)
#    build       compile only -> $(SIM_BIN)
#    compile     alias of build
#    wave        run, then open the VCD in gtkwave
#    lint        parse + elaborate only, report warnings
#    clean       remove $(BUILD_DIR)/
#    help        show this message
#
#  Variables (override on the command line)
#    TOP         testbench module / file stem        (default: pe_tb)
#    RTL_DIR     synthesizable sources               (default: ../rtl)
#    TB_DIR      testbench sources                   (default: ../tb)
#    BUILD_DIR   build output directory              (default: build)
#    EXTRA_SRCS  extra sources appended to the build
#    IVFLAGS     iverilog flags
#
#  Source selection (automatic)
#    testbench : tb/**/<TOP>.sv (or .v)
#    RTL       : rtl/**/<stem>.sv where <stem> = <TOP> minus the "_tb" suffix
#                (e.g. pe_tb -> pe.sv, delay_chain_tb -> delay_chain.sv); if no
#                such file exists, every RTL source is used instead. This keeps
#                a testbench buildable even when an unrelated source in the tree
#                does not compile yet.
#    Override  : drop a source list in sim/<TOP>.files (one path per line).
#                Use it for testbenches needing several RTL files, e.g. MXU.
# =============================================================================

# ---- Tools ------------------------------------------------------------------
IV      ?= iverilog
VVP     ?= vvp
WAVE    ?= gtkwave

# ---- Paths ------------------------------------------------------------------
TOP       ?= pe_tb
RTL_DIR   ?= ../rtl
TB_DIR    ?= ../tb
BUILD_DIR ?= build
FILES_DIR ?= .

SIM_BIN   := $(BUILD_DIR)/$(TOP).vvp
WAVE_FILE := $(BUILD_DIR)/$(TOP).vcd

# ---- Compile flags ----------------------------------------------------------
IVFLAGS ?= -g2012 -Wall -Wno-timescale
IVFLAGS += -s $(TOP)                      # force testbench as the root module

# ---- Auto-discovered testbenches (tb/**/*_tb.sv -> "xxx_tb") ----------------
TB_TOPS := $(basename $(notdir $(wildcard $(TB_DIR)/*_tb.sv $(TB_DIR)/*/*_tb.sv)))

# ---- Source selection for $(TOP) --------------------------------------------
STEM      := $(patsubst %_tb,%,$(TOP))
RTL_ALL   := $(sort $(wildcard $(RTL_DIR)/*.sv  $(RTL_DIR)/*/*.sv \
                              $(RTL_DIR)/*.v   $(RTL_DIR)/*/*.v))
RTL_THIS  := $(sort $(wildcard $(RTL_DIR)/$(STEM).sv  $(RTL_DIR)/*/$(STEM).sv \
                              $(RTL_DIR)/$(STEM).v   $(RTL_DIR)/*/$(STEM).v))
TOP_FILES := $(FILES_DIR)/$(TOP).files
TB_SRCS   := $(sort $(wildcard $(TB_DIR)/$(TOP).sv  $(TB_DIR)/*/$(TOP).sv \
                              $(TB_DIR)/$(TOP).v   $(TB_DIR)/*/$(TOP).v))

ifeq ($(wildcard $(TOP_FILES)),)
  RTL_SRCS := $(if $(RTL_THIS),$(RTL_THIS),$(RTL_ALL))
else
  RTL_SRCS := $(shell cat $(TOP_FILES))
endif

SRCS := $(RTL_SRCS) $(TB_SRCS) $(EXTRA_SRCS)

ifeq ($(strip $(TB_SRCS)),)
  $(warning no testbench tb/**/$(TOP).sv found for TOP=$(TOP))
endif

# =============================================================================
.DEFAULT_GOAL := all
.PHONY: all run build compile wave lint regress list clean help $(TB_TOPS)

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
	@echo ">> lint OK ($(TOP))"

# ---- Run every discovered testbench -----------------------------------------
regress: $(TB_TOPS)

$(TB_TOPS):
	@$(MAKE) --no-print-directory TOP=$@ run

# ---- List discovered testbenches --------------------------------------------
list:
	@echo "testbenches under $(TB_DIR):"
	@for t in $(TB_TOPS); do \
	  if [ -f $(FILES_DIR)/$$t.files ]; then \
	    rtl=$$(tr '\n' ' ' < $(FILES_DIR)/$$t.files); \
	  else \
	    stem=$${t%_tb}; \
	    rtl=$$(ls $(RTL_DIR)/$$stem.sv $(RTL_DIR)/*/$$stem.sv 2>/dev/null | tr '\n' ' '); \
	    [ -z "$$rtl" ] && rtl="<all rtl>"; \
	  fi; \
	  printf "  %-18s rtl : %s\n" "$$t" "$$rtl"; \
	done

# ---- Clean ------------------------------------------------------------------
clean:
	rm -rf $(BUILD_DIR)

# ---- Help -------------------------------------------------------------------
help:
	@echo "TPUv1 simulation"
	@echo ""
	@echo "  make                 compile + run TOP ($(TOP))"
	@echo "  make <tb>            compile + run an auto-discovered testbench"
	@echo "  make build           compile only -> $(SIM_BIN)"
	@echo "  make wave            run + open $(WAVE_FILE) in gtkwave"
	@echo "  make lint            elaborate only, report warnings"
	@echo "  make regress         run every discovered testbench"
	@echo "  make list            list discovered testbenches"
	@echo "  make clean           remove $(BUILD_DIR)/"
	@echo ""
	@echo "  testbenches : $(TB_TOPS)"
	@echo "  rtl sources : $(RTL_SRCS)"
	@echo "  tb source   : $(TB_SRCS)"
