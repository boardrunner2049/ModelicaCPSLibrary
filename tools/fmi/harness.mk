# Shared build rules for the FIRE FMI harnesses.
#
# A harness directory's Makefile sets MODEL (the library class without the
# ModelicaCPSLibrary. prefix) and includes this file. Optional:
#   WRAPPER, WRAPPER_MODEL  compile a harness-local Modelica wrapper instead
#                           (for components that cannot stand alone)
#   EXPECT_REFUSAL          the model must be refused at export with a message
#                           containing this text (deliberately invalid models)
#
# Targets: all (build), run (simulate + checks), view (trace observer), clean.
#
# Machine settings (MODELICA_LIBS, WORLD_ROOT, RUMOCA, ...) come from the
# environment or tools/fmi/local.env; see local.env.example.

FIRE_FMI := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
-include $(FIRE_FMI)/local.env
export MODELICA_LIBS RUMOCA FIRE_FMI_CACHE FMI3_HEADERS
WORLD_ROOT ?= $(HOME)/Documents/world
PYTHON ?= $(WORLD_ROOT)/venv/bin/python
CC ?= cc

OUT := out
FMU := $(OUT)/model.fmu
HARNESS := $(OUT)/harness
GENERATED := $(OUT)/.generated
WORLD_LIB := $(WORLD_ROOT)/build/libworld_model.so
CFLAGS := -std=c17 -O2 -Wall -Wextra -Wpedantic -I$(WORLD_ROOT)/include -I$(FIRE_FMI)/lib -I$(OUT)

.PHONY: all run view clean

ifdef EXPECT_REFUSAL
all:
run:
	@$(PYTHON) $(FIRE_FMI)/tools/expect_refusal.py $(MODEL) "$(EXPECT_REFUSAL)"
clean:
	rm -rf $(OUT)
else
all: $(HARNESS)

$(FMU): $(WRAPPER) $(wildcard $(FIRE_FMI)/bin/rumoca)
	@mkdir -p $(OUT)
	$(PYTHON) $(FIRE_FMI)/tools/make_fmu.py $(MODEL) $@ $(if $(WRAPPER),--wrapper $(WRAPPER) --wrapper-model $(WRAPPER_MODEL))

$(GENERATED): world.toml $(FMU)
	$(PYTHON) $(WORLD_ROOT)/tools/wm_generate.py world.toml
	@touch $@

$(OUT)/world_bindings.c $(OUT)/world_bindings.h: $(GENERATED)

$(HARNESS): harness.c $(FIRE_FMI)/lib/fire_harness.c $(FIRE_FMI)/lib/fire_harness.h $(GENERATED)
	$(CC) $(CFLAGS) -o $@ harness.c $(FIRE_FMI)/lib/fire_harness.c $(OUT)/world_bindings.c \
	  -L$(dir $(WORLD_LIB)) -lworld_model -lm -Wl,-rpath,$(dir $(WORLD_LIB))

run: $(HARNESS)
	./$(HARNESS)
	$(PYTHON) $(FIRE_FMI)/tools/check_trace.py checks.toml $(OUT)/trace.csv

view: $(GENERATED)
	$(PYTHON) $(WORLD_ROOT)/tools/wm_view.py $(OUT)

clean:
	rm -rf $(OUT)
endif
