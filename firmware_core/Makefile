# ============================================================================
# firmware_core/Makefile
#
# Pure C99, zero-dependency algorithm layer. All source files may only use <stdint.h> / <stddef.h> /
# <string.h> and our own headers; no HAL / ROS2 / third-party / <stdio.h> /
# <math.h>; all-integer, zero floating-point.
#
# Common targets
#   make test          build and run all tests (each test file prints its own case count)
#   make test-asan     run everything again under ASan + UBSan
#   make check-purity  purity gate: include whitelist + banned library calls + banned floating-point
#   make check         compile every source file on its own (warnings are errors)
#   make lib           produce build/libomninav_core.so and self-check the frozen symbols
#   make clean
#
# ★ Source / header / test files are all found with wildcard (an older version hard-coded single files,
#   which made check-purity miss new modules — exactly what acceptance criterion 3 guards against)
# ============================================================================

CC ?= cc

WARNFLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -Wconversion -Wshadow -Wcast-qual
CFLAGS ?= $(WARNFLAGS) -O2
ASANFLAGS = $(WARNFLAGS) -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer

INC = -Iinclude

# libm for tests (only tests may use floating-point; src/ and include/ are strictly zero floating-point)
TEST_LDLIBS = -lm

BUILD = build

SRC  = $(sort $(wildcard src/*.c))
HDR  = $(sort $(wildcard include/*.h))
TEST = $(sort $(wildcard test/*.c))

SRC_OBJ   = $(patsubst src/%.c,$(BUILD)/%.o,$(SRC))
ASAN_OBJ  = $(patsubst src/%.c,$(BUILD)/asan/%.o,$(SRC))
LIB       = $(BUILD)/libomninav_core.so
TEST_BINS      = $(patsubst test/%.c,$(BUILD)/%,$(TEST))
TEST_BINS_ASAN = $(patsubst test/%.c,$(BUILD)/%_asan,$(TEST))

# Contract §5.2: these 7 symbols must appear in the dynamic symbol table of libomninav_core.so
LIB_SYMBOLS = mecanum_forward mecanum_inverse mecanum_l_mm \
              mecanum_default_geom mecanum_geom_is_valid \
              odom_update odom_reset

# ---- Regexes used by the purity gate (note: no '#' when matching include,
#      only include< or include", so explanatory sentences in comments are covered too) ----
SCAN_INCLUDE = include[[:space:]]*[<"]
OK_INCLUDE   = "(mecanum|pid|protocol|odom|fixedpoint)\.h"|<(stdint|stddef|string)\.h>
FORBIDDEN    = (float|double|printf|malloc|free|memcpy|memset|sqrt|sin|cos|atan2)

.PHONY: all test test-asan check-purity check lib clean help

all: check test lib

# ---------------------------------------------------------------- Build

$(BUILD)/%.o: src/%.c $(HDR)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INC) -fPIC -c $< -o $@

$(BUILD)/asan/%.o: src/%.c $(HDR)
	@mkdir -p $(dir $@)
	$(CC) $(ASANFLAGS) $(INC) -c $< -o $@

$(BUILD)/%: test/%.c $(SRC_OBJ) $(HDR)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INC) $< $(SRC_OBJ) -o $@ $(TEST_LDLIBS)

$(BUILD)/%_asan: test/%.c $(ASAN_OBJ) $(HDR)
	@mkdir -p $(dir $@)
	$(CC) $(ASANFLAGS) $(INC) $< $(ASAN_OBJ) -o $@ $(TEST_LDLIBS)

check: $(SRC_OBJ)
	@echo "check: all $(words $(SRC)) source files compile standalone (-Werror)"

# ---------------------------------------------------------------- Tests

test: $(TEST_BINS)
	@echo "===================== UNIT TESTS ====================="
	@fail=0; \
	for t in $(TEST_BINS); do \
	    echo "---- $$t ----"; \
	    ./$$t || fail=1; \
	done; \
	echo "==================================================="; \
	if [ $$fail -ne 0 ]; then echo "TEST: FAILED"; exit 1; fi; \
	echo "TEST: ALL PASSED ($(words $(TEST_BINS)) test files)"

test-asan: $(TEST_BINS_ASAN)
	@echo "============== UNIT TESTS (ASan + UBSan) =============="
	@fail=0; \
	for t in $(TEST_BINS_ASAN); do \
	    echo "---- $$t ----"; \
	    ASAN_OPTIONS=detect_leaks=1 ./$$t || fail=1; \
	done; \
	echo "==================================================="; \
	if [ $$fail -ne 0 ]; then echo "TEST-ASAN: FAILED"; exit 1; fi; \
	echo "TEST-ASAN: ALL PASSED ($(words $(TEST_BINS_ASAN)) test files)"

# ---------------------------------------------------------------- Purity gate

check-purity:
	@echo "== check-purity: checking $(words $(SRC)) .c + $(words $(HDR)) .h files =="
	@bad=0; n=0; \
	for f in $(SRC) $(HDR); do \
	    n=$$((n+1)); \
	    echo "  [scan] $$f"; \
	    inc=`grep -hE '$(SCAN_INCLUDE)' $$f 2>/dev/null | grep -vE '$(OK_INCLUDE)' || true`; \
	    if [ -n "$$inc" ]; then \
	        echo "  [VIOLATION] $$f has a disallowed include:"; \
	        echo "$$inc" | sed 's/^/         /'; \
	        bad=1; \
	    fi; \
	    w=`grep -nwE '$(FORBIDDEN)' $$f 2>/dev/null || true`; \
	    if [ -n "$$w" ]; then \
	        echo "  [VIOLATION] $$f contains a floating-point / library-call identifier:"; \
	        echo "$$w" | sed 's/^/         /'; \
	        bad=1; \
	    fi; \
	    m=`echo "$$f" | grep -E '^src/' || true`; \
	    if [ -n "$$m" ]; then \
	        mm=`grep -nwE 'main' $$f 2>/dev/null || true`; \
	        if [ -n "$$mm" ]; then \
	            echo "  [VIOLATION] $$f must not contain main:"; \
	            echo "$$mm" | sed 's/^/         /'; \
	            bad=1; \
	        fi; \
	    fi; \
	done; \
	echo "  [coverage] scanned $$n files (src: $(words $(SRC)), include: $(words $(HDR)))"; \
	if [ $$bad -ne 0 ]; then echo "check-purity: FAILED"; exit 1; fi; \
	echo "check-purity: PASSED"

# ---------------------------------------------------------------- Shared library

$(LIB): $(SRC_OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $(SRC_OBJ)

lib: $(LIB)
	@echo "===================== SHARED LIB ====================="
	@echo "-- nm -D --defined-only $(LIB) --"
	@nm -D --defined-only $(LIB) | sort
	@echo "-- contract §5.2 frozen-symbol self-check --"
	@fail=0; \
	for s in $(LIB_SYMBOLS); do \
	    if nm -D --defined-only $(LIB) | grep -qw "$$s"; then \
	        echo "  [ok]   $$s"; \
	    else \
	        echo "  [MISSING] $$s"; fail=1; \
	    fi; \
	done; \
	if [ $$fail -ne 0 ]; then echo "LIB: FAILED"; exit 1; fi; \
	echo "LIB: PASSED -> $(LIB)"

# ---------------------------------------------------------------- Misc

clean:
	rm -rf $(BUILD)

help:
	@echo "make test | test-asan | check | check-purity | lib | clean"
	@echo "sources: $(SRC)"
	@echo "headers: $(HDR)"
	@echo "tests  : $(TEST)"
