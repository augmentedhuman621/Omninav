# ============================================================================
# firmware_core/Makefile
#
# 纯 C99、零依赖的算法层。所有源文件只允许 <stdint.h> / <stddef.h> /
# <string.h> 和自己人的头文件；不许 HAL / ROS2 / 第三方 / <stdio.h> /
# <math.h>；全整数零浮点。
#
# 常用目标
#   make test          编译并运行全部测试（每个测试文件自己打印用例数）
#   make test-asan     ASan + UBSan 再跑一遍
#   make check-purity  纯度门禁：include 白名单 + 禁库函数 + 禁浮点
#   make check         全部源文件单独编译一遍（警告即错误）
#   make lib           产出 build/libomninav_core.so 并自检冻结符号
#   make clean
#
# ★ 源文件/头文件/测试文件全部用 wildcard 扫描（旧版写死了单个文件，
#   会让 check-purity 漏查新模块 —— 这正是验收标准第 3 条要防的）
# ============================================================================

CC ?= cc

WARNFLAGS = -std=c99 -Wall -Wextra -Werror -pedantic -Wconversion -Wshadow -Wcast-qual
CFLAGS ?= $(WARNFLAGS) -O2
ASANFLAGS = $(WARNFLAGS) -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer

INC = -Iinclude

# 测试用 libm（只有测试允许浮点，src/ 与 include/ 一律零浮点）
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

# 契约 §5.2：这 7 个符号必须出现在 libomninav_core.so 的动态符号表里
LIB_SYMBOLS = mecanum_forward mecanum_inverse mecanum_l_mm \
              mecanum_default_geom mecanum_geom_is_valid \
              odom_update odom_reset

# ---- 纯度门禁用的正则（注意：匹配 include 时不写 '#'，
#      只用 include<或include"，这样注释里的说明句也一并覆盖）----
SCAN_INCLUDE = include[[:space:]]*[<"]
OK_INCLUDE   = "(mecanum|pid|protocol|odom|fixedpoint)\.h"|<(stdint|stddef|string)\.h>
FORBIDDEN    = (float|double|printf|malloc|free|memcpy|memset|sqrt|sin|cos|atan2)

.PHONY: all test test-asan check-purity check lib clean help

all: check test lib

# ---------------------------------------------------------------- 编译

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
	@echo "check: $(words $(SRC)) 个源文件全部单独编译通过（-Werror）"

# ---------------------------------------------------------------- 测试

test: $(TEST_BINS)
	@echo "===================== 单元测试 ====================="
	@fail=0; \
	for t in $(TEST_BINS); do \
	    echo "---- $$t ----"; \
	    ./$$t || fail=1; \
	done; \
	echo "==================================================="; \
	if [ $$fail -ne 0 ]; then echo "TEST: FAILED"; exit 1; fi; \
	echo "TEST: ALL PASSED（$(words $(TEST_BINS)) 个测试文件）"

test-asan: $(TEST_BINS_ASAN)
	@echo "============== 单元测试（ASan + UBSan）=============="
	@fail=0; \
	for t in $(TEST_BINS_ASAN); do \
	    echo "---- $$t ----"; \
	    ASAN_OPTIONS=detect_leaks=1 ./$$t || fail=1; \
	done; \
	echo "==================================================="; \
	if [ $$fail -ne 0 ]; then echo "TEST-ASAN: FAILED"; exit 1; fi; \
	echo "TEST-ASAN: ALL PASSED（$(words $(TEST_BINS_ASAN)) 个测试文件）"

# ---------------------------------------------------------------- 纯度门禁

check-purity:
	@echo "== check-purity: 待查 $(words $(SRC)) 个 .c + $(words $(HDR)) 个 .h =="
	@bad=0; n=0; \
	for f in $(SRC) $(HDR); do \
	    n=$$((n+1)); \
	    echo "  [扫] $$f"; \
	    inc=`grep -hE '$(SCAN_INCLUDE)' $$f 2>/dev/null | grep -vE '$(OK_INCLUDE)' || true`; \
	    if [ -n "$$inc" ]; then \
	        echo "  [违约] $$f 有不被允许的 include:"; \
	        echo "$$inc" | sed 's/^/         /'; \
	        bad=1; \
	    fi; \
	    w=`grep -nwE '$(FORBIDDEN)' $$f 2>/dev/null || true`; \
	    if [ -n "$$w" ]; then \
	        echo "  [违约] $$f 出现浮点/库函数标识符:"; \
	        echo "$$w" | sed 's/^/         /'; \
	        bad=1; \
	    fi; \
	    m=`echo "$$f" | grep -E '^src/' || true`; \
	    if [ -n "$$m" ]; then \
	        mm=`grep -nwE 'main' $$f 2>/dev/null || true`; \
	        if [ -n "$$mm" ]; then \
	            echo "  [违约] $$f 里不应出现 main:"; \
	            echo "$$mm" | sed 's/^/         /'; \
	            bad=1; \
	        fi; \
	    fi; \
	done; \
	echo "  [覆盖] 共扫描 $$n 个文件（src: $(words $(SRC)) 个，include: $(words $(HDR)) 个）"; \
	if [ $$bad -ne 0 ]; then echo "check-purity: FAILED"; exit 1; fi; \
	echo "check-purity: PASSED"

# ---------------------------------------------------------------- 共享库

$(LIB): $(SRC_OBJ)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -fPIC -shared -o $@ $(SRC_OBJ)

lib: $(LIB)
	@echo "===================== 共享库 ====================="
	@echo "-- nm -D --defined-only $(LIB) --"
	@nm -D --defined-only $(LIB) | sort
	@echo "-- 契约 §5.2 冻结符号自检 --"
	@fail=0; \
	for s in $(LIB_SYMBOLS); do \
	    if nm -D --defined-only $(LIB) | grep -qw "$$s"; then \
	        echo "  [ok]   $$s"; \
	    else \
	        echo "  [缺失] $$s"; fail=1; \
	    fi; \
	done; \
	if [ $$fail -ne 0 ]; then echo "LIB: FAILED"; exit 1; fi; \
	echo "LIB: PASSED -> $(LIB)"

# ---------------------------------------------------------------- 杂项

clean:
	rm -rf $(BUILD)

help:
	@echo "make test | test-asan | check | check-purity | lib | clean"
	@echo "源文件: $(SRC)"
	@echo "头文件: $(HDR)"
	@echo "测试  : $(TEST)"
