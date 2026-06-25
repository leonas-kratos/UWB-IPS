###############################################################################
# Top-level Makefile for DWM1001 Examples
#
# Usage:
#   make ss_bridge        - Compile ss_bridge project
#   make ss_twr_init      - Compile ss_twr_init project
#   make ss_twr_resp      - Compile ss_twr_resp project
#   make all              - Compile all projects
#
#   make flash_ss_bridge  - Flash ss_bridge to device
#   make flash_ss_twr_init - Flash ss_twr_init to device
#   make flash_ss_twr_resp - Flash ss_twr_resp to device
#
#   make clean            - Clean all build outputs
#   make clean_ss_bridge  - Clean only ss_bridge
#   make clean_ss_twr_init - Clean only ss_twr_init
#   make clean_ss_twr_resp - Clean only ss_twr_resp
###############################################################################

EXAMPLES_DIR := examples

# List of all projects
PROJECTS := ss_bridge ss_twr_init ss_twr_resp

.PHONY: all clean help $(PROJECTS) \
        $(addprefix flash_, $(PROJECTS)) \
        $(addprefix clean_, $(PROJECTS)) \
        $(addprefix flash_jlink_, $(PROJECTS))

# Default: show help
help:
	@echo ""
	@echo "╔══════════════════════════════════════════════════════════╗"
	@echo "║          DWM1001 Examples - Build System                ║"
	@echo "╠══════════════════════════════════════════════════════════╣"
	@echo "║                                                        ║"
	@echo "║  Build:                                                 ║"
	@echo "║    make ss_bridge         Compile Bridge project        ║"
	@echo "║    make ss_twr_init       Compile TWR Initiator         ║"
	@echo "║    make ss_twr_resp       Compile TWR Responder         ║"
	@echo "║    make all               Compile all projects          ║"
	@echo "║                                                        ║"
	@echo "║  Flash (nrfjprog):                                      ║"
	@echo "║    make flash_ss_bridge   Flash Bridge to device        ║"
	@echo "║    make flash_ss_twr_init Flash TWR Initiator           ║"
	@echo "║    make flash_ss_twr_resp Flash TWR Responder           ║"
	@echo "║                                                        ║"
	@echo "║  Flash (JLink):                                         ║"
	@echo "║    make flash_jlink_ss_bridge    Flash via JLink        ║"
	@echo "║    make flash_jlink_ss_twr_init  Flash via JLink        ║"
	@echo "║    make flash_jlink_ss_twr_resp  Flash via JLink        ║"
	@echo "║                                                        ║"
	@echo "║  Clean:                                                 ║"
	@echo "║    make clean             Clean all projects            ║"
	@echo "║    make clean_ss_bridge   Clean Bridge only             ║"
	@echo "║    make clean_ss_twr_init Clean TWR Initiator only      ║"
	@echo "║    make clean_ss_twr_resp Clean TWR Responder only      ║"
	@echo "║                                                        ║"
	@echo "╚══════════════════════════════════════════════════════════╝"
	@echo ""

# Build all
all: $(PROJECTS)

# Build individual projects
ss_bridge:
	@echo "========== Building ss_bridge =========="
	$(MAKE) -C $(EXAMPLES_DIR)/ss_bridge all

ss_twr_init:
	@echo "========== Building ss_twr_init =========="
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_init all

ss_twr_resp:
	@echo "========== Building ss_twr_resp =========="
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_resp all

# Flash individual projects (nrfjprog)
flash_ss_bridge: ss_bridge
	$(MAKE) -C $(EXAMPLES_DIR)/ss_bridge flash

flash_ss_twr_init: ss_twr_init
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_init flash

flash_ss_twr_resp: ss_twr_resp
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_resp flash

# Flash individual projects (JLink Commander)
flash_jlink_ss_bridge: ss_bridge
	$(MAKE) -C $(EXAMPLES_DIR)/ss_bridge flash_jlink

flash_jlink_ss_twr_init: ss_twr_init
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_init flash_jlink

flash_jlink_ss_twr_resp: ss_twr_resp
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_resp flash_jlink

# Clean all
clean:
	@for proj in $(PROJECTS); do \
		echo "========== Cleaning $$proj =========="; \
		$(MAKE) -C $(EXAMPLES_DIR)/$$proj clean; \
	done

# Clean individual projects
clean_ss_bridge:
	$(MAKE) -C $(EXAMPLES_DIR)/ss_bridge clean

clean_ss_twr_init:
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_init clean

clean_ss_twr_resp:
	$(MAKE) -C $(EXAMPLES_DIR)/ss_twr_resp clean

# Erase all chip flash
erase:
	nrfjprog -f nrf52 --eraseall
