# ============================================================
#  Makefile — UWB Tracking + CIR Plotter (C shared libraries)
#
#  Targets:
#    make              → build tất cả
#    make uwb          → chỉ build uwb_tracking.so/.dll
#    make cir          → chỉ build cir_processor.so/.dll
#    make clean        → xoá file build
#    make help         → hiện hướng dẫn
# ============================================================

CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -fPIC
LDFLAGS = -lm

# ── Tự động nhận diện hệ điều hành ──────────────────────
ifeq ($(OS), Windows_NT)
    EXT     = .dll
    SHARED  = -shared
    RM      = del /Q
else
    UNAME := $(shell uname -s)
    ifeq ($(UNAME), Darwin)          # macOS
        EXT    = .dylib
        SHARED = -dynamiclib
    else                             # Linux
        EXT    = .so
        SHARED = -shared
    endif
    RM = rm -f
endif

# ── Tên file output ──────────────────────────────────────
UWB_SRC = uwb_tracking.c
UWB_OUT = uwb_tracking$(EXT)

CIR_SRC = cir_processor.c
CIR_OUT = cir_processor$(EXT)

# ============================================================
.PHONY: all uwb cir clean help

all: uwb cir
	@echo ""
	@echo "✓ Build xong:"
	@echo "  $(UWB_OUT)"
	@echo "  $(CIR_OUT)"

uwb: $(UWB_OUT)

$(UWB_OUT): $(UWB_SRC)
	$(CC) $(CFLAGS) $(SHARED) -o $@ $< $(LDFLAGS)
	@echo "✓ $(UWB_OUT)"

cir: $(CIR_OUT)

$(CIR_OUT): $(CIR_SRC)
	$(CC) $(CFLAGS) $(SHARED) -o $@ $< $(LDFLAGS)
	@echo "✓ $(CIR_OUT)"

clean:
	$(RM) $(UWB_OUT) $(CIR_OUT)
	@echo "✓ Cleaned"

help:
	@echo "Cách dùng:"
	@echo "  make          — build uwb + cir"
	@echo "  make uwb      — chỉ build uwb_tracking$(EXT)"
	@echo "  make cir      — chỉ build cir_processor$(EXT)"
	@echo "  make clean    — xoá file build"
