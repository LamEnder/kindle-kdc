# kdc - Kindle PW4 pulse-drive display controller
#
#   make            host build (simulator) + tests
#   make test       run host tests (WFM=file.wrf also patches a real waveform)
#   make kindle     cross build for the Kindle (kindlehf toolchain)
#   make dist       kindle build + scripts in build/kindle/kdc/

KINDLE_TC ?= $(abspath ../x-tools/arm-kindlehf-linux-gnueabihf/bin)
CROSS     ?= $(KINDLE_TC)/arm-kindlehf-linux-gnueabihf-

SRC  := src/kdc.c src/kdc_plan.c src/kdc_dev.c src/kdc_wfm.c
HDRS := include/kdc.h src/kdc_plan.h src/kdc_dev.h src/kdc_wfm.h src/mxcfb_pw4.h
WARN := -Wall -Wextra -Wshadow -Wno-unused-parameter
BASE := -std=gnu11 -Iinclude -Isrc $(WARN)

HOST_CC     ?= cc
HOST_CFLAGS ?= -O2 -g -fsanitize=address,undefined -fno-omit-frame-pointer
KINDLE_CFLAGS ?= -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -mthumb -ffast-math -pipe

H := build/host
K := build/kindle

.PHONY: all host test kindle dist vecreport clean
all: test

host: $(H)/test_host $(H)/kdc-wfm $(H)/kdc-demo $(H)/kdc-cal $(H)/kdc-bench

$(H)/libkdc.a: $(SRC) $(HDRS) | $(H)
	cd $(H) && $(HOST_CC) $(BASE:-I%=-I$(CURDIR)/%) $(HOST_CFLAGS) -c $(addprefix $(CURDIR)/,$(SRC))
	ar rcs $@ $(addprefix $(H)/,$(notdir $(SRC:.c=.o)))

$(H)/%: tools/%.c $(H)/libkdc.a
	$(HOST_CC) $(BASE) $(HOST_CFLAGS) $< $(H)/libkdc.a -lpthread -lm -o $@

$(H)/test_host: tests/test_host.c $(H)/libkdc.a
	$(HOST_CC) $(BASE) $(HOST_CFLAGS) $< $(H)/libkdc.a -lpthread -lm -o $@

test: host
	$(H)/test_host $(WFM)

kindle: $(K)/kdc-wfm $(K)/kdc-demo $(K)/kdc-cal $(K)/kdc-bench $(K)/libkdc.a

$(K)/libkdc.a: $(SRC) $(HDRS) | $(K)
	cd $(K) && $(CROSS)gcc $(BASE:-I%=-I$(CURDIR)/%) $(KINDLE_CFLAGS) -c $(addprefix $(CURDIR)/,$(SRC))
	$(CROSS)ar rcs $@ $(addprefix $(K)/,$(notdir $(SRC:.c=.o)))

$(K)/%: tools/%.c $(K)/libkdc.a
	$(CROSS)gcc $(BASE) $(KINDLE_CFLAGS) $< $(K)/libkdc.a -static-libgcc -lpthread -lm -o $@
	$(CROSS)strip $@

dist: kindle
	mkdir -p $(K)/kdc
	cp $(K)/kdc-wfm $(K)/kdc-demo $(K)/kdc-cal scripts/kdc-pulse.sh scripts/kdc-session.sh $(K)/kdc/
	chmod +x $(K)/kdc/*
	cd $(K) && tar czf kdc-kindle.tar.gz kdc
	@echo "-> $(K)/kdc-kindle.tar.gz"

# Shows which hot loops NEON-vectorized.
vecreport: | $(K)
	$(CROSS)gcc $(BASE) $(KINDLE_CFLAGS) -fopt-info-vec-optimized -c src/kdc_plan.c -o /dev/null

$(H) $(K):
	mkdir -p $@

clean:
	rm -rf build
