# Generic vehicle harness: fly any fire-compose multirotor configuration.
#
#   make VEHICLE=configs/quad.toml run     # emit, export, build, fly, check
#   make VEHICLE=configs/hex_p2.toml all   # build only
#   make VEHICLE=... view | clean
#
# VEHICLE is a fire-compose config (relative to this directory or to the
# library root); MISSION (default mission.toml) and CHECKS (default
# checks.template.toml) select the scenario. Everything is built in
# out/<config name>/, an ordinary harness directory driven by ../harness.mk;
# fire-compose emits into <library>/build/fmi-vehicle/<config name>/.

VEHICLE ?= configs/quad.toml
MISSION ?= mission.toml
CHECKS ?= checks.template.toml

FIRE_FMI := $(abspath ..)
-include $(FIRE_FMI)/local.env
WORLD_ROOT ?= $(HOME)/Documents/world
PYTHON ?= $(WORLD_ROOT)/venv/bin/python

NAME := $(basename $(notdir $(VEHICLE)))
DIR := out/$(NAME)

.PHONY: prepare all run view clean

prepare:
	@$(PYTHON) tools/vehicle.py prepare $(VEHICLE) --mission $(MISSION) --checks $(CHECKS) --out $(DIR)

all run view: prepare
	@$(MAKE) --no-print-directory -C $(DIR) $@

clean:
	rm -rf $(DIR)
