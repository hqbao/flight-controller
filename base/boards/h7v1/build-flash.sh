#!/bin/bash
# Build, flash, and optionally debug STM32H7 Flight Controller
# Usage:
#   ./build-flash.sh              # Build and flash
#   ./build-flash.sh --verify     # Build, flash, and verify
#   ./build-flash.sh --debug      # Build, flash, and debug with GDB
#   ./build-flash.sh --verify --debug  # All three

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEBUG_DIR="$PROJECT_DIR/Debug"
BINARY="$DEBUG_DIR/h7v1.elf"
BIN_FILE="$DEBUG_DIR/h7v1.bin"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Parse arguments
VERIFY=false
DEBUG=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --verify) VERIFY=true; shift ;;
        --debug) DEBUG=true; shift ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}STM32H7 Flight Controller${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Step 1: Build
echo -e "${YELLOW}[1/2] Building...${NC}"
"$PROJECT_DIR/build.sh"
echo ""

# Step 2: Flash
echo -e "${YELLOW}[2/2] Flashing to device...${NC}"

if [[ ! -f "$BIN_FILE" ]]; then
    echo -e "${RED}✗ Binary not found: $BIN_FILE${NC}"
    exit 1
fi

if ! st-info --probe &> /dev/null; then
    echo -e "${RED}✗ ST-Link debugger not found${NC}"
    exit 1
fi

echo -e "${YELLOW}Flashing to 0x8000000...${NC}"
st-flash write "$BIN_FILE" 0x8000000

if [[ $? -eq 0 ]]; then
    echo -e "${GREEN}✓ Flash successful${NC}"
    
    if [[ "$VERIFY" == true ]]; then
        echo -e "${YELLOW}Verifying flash...${NC}"
        st-flash verify "$BIN_FILE" 0x8000000
        echo -e "${GREEN}✓ Verification successful${NC}"
    fi
else
    echo -e "${RED}✗ Flash failed${NC}"
    exit 1
fi

echo ""

# Step 3: Debug (if requested)
if [[ "$DEBUG" == true ]]; then
    echo -e "${YELLOW}[3/3] Starting debugger...${NC}"
    echo ""
    
    # Check if OpenOCD is installed
    if ! command -v openocd &> /dev/null; then
        echo -e "${RED}✗ OpenOCD not found${NC}"
        echo -e "${YELLOW}Install with: brew install openocd${NC}"
        exit 1
    fi
    
    # Start OpenOCD in background
    echo -e "${YELLOW}Starting OpenOCD...${NC}"
    openocd -f interface/stlink.cfg -f target/stm32h7x.cfg > /tmp/openocd.log 2>&1 &
    OPENOCD_PID=$!
    
    # Wait for server to start
    sleep 2
    
    # Verify OpenOCD started
    if ! kill -0 $OPENOCD_PID 2>/dev/null; then
        echo -e "${RED}✗ OpenOCD failed to start${NC}"
        echo -e "${YELLOW}Check /tmp/openocd.log${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ OpenOCD started (PID: $OPENOCD_PID)${NC}"
    echo -e "${YELLOW}OpenOCD log: /tmp/openocd.log${NC}"
    
    # Set trap to kill OpenOCD on exit
    trap "kill $OPENOCD_PID 2>/dev/null; echo -e '${YELLOW}OpenOCD stopped${NC}'" EXIT
    
    echo ""
    echo -e "${BLUE}═══════════════════════════════════════${NC}"
    echo -e "${BLUE}GDB Debugger Tips:${NC}"
    echo -e "${BLUE}───────────────────────────────────────${NC}"
    echo -e "  ${GREEN}break${NC} <function>           Set breakpoint"
    echo -e "  ${GREEN}break${NC} file.c:100             Set breakpoint at line"
    echo -e "  ${GREEN}continue${NC}                     Run until breakpoint"
    echo -e "  ${GREEN}step${NC}                         Step into function"
    echo -e "  ${GREEN}next{{NC}}                         Step over function"
    echo -e "  ${GREEN}print{{NC}} <var>                  Print variable"
    echo -e "  ${GREEN}display{{NC}} <var>                Auto-print on step"
    echo -e "  ${GREEN}backtrace{{NC}}                    Show call stack"
    echo -e "  ${GREEN}quit{{NC}}                         Exit debugger"
    echo -e "${BLUE}═══════════════════════════════════════${NC}"
    echo ""
    
    if [[ ! -f "$BINARY" ]]; then
        echo -e "${RED}✗ Binary not found: $BINARY${NC}"
        exit 1
    fi
    
    # Launch GDB
    arm-none-eabi-gdb "$BINARY" \
        -ex "target remote localhost:3333" \
        -ex "monitor reset halt" \
        -ex "load" \
        -ex "break main" \
        -ex "continue"
    
    echo ""
    echo -e "${YELLOW}========================================${NC}"
    echo -e "${GREEN}✓ Debug session ended${NC}"
    echo -e "${YELLOW}========================================${NC}"
else
    echo -e "${YELLOW}========================================${NC}"
    echo -e "${GREEN}✓ Build + Flash Complete${NC}"
    echo -e "${YELLOW}========================================${NC}"
fi
