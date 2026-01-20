#!/bin/bash
# Build script for STM32H7 Flight Controller
# Compiles the flight controller firmware
# Usage: ./build.sh [clean]

set -e

# Fix for macOS: Prioritize Homebrew Cask toolchain over STM32CubeIDE if incompatible
if [[ "$OSTYPE" == "darwin"* ]]; then
    CASK_GCC=$(ls -d /Applications/ArmGNUToolchain/*/arm-none-eabi/bin 2>/dev/null | head -n 1)
    if [[ -d "$CASK_GCC" ]]; then
        export PATH="$CASK_GCC:$PATH"
        # echo -e "Using Toolchain: $CASK_GCC"
    fi
fi

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEBUG_DIR="$PROJECT_DIR/Debug"
BINARY="$DEBUG_DIR/h7v1.elf"
BIN_FILE="$DEBUG_DIR/h7v1.bin"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}STM32H7 Flight Controller - Build${NC}"
echo -e "${YELLOW}========================================${NC}"

# Clean if requested
if [[ "$1" == "clean" ]]; then
    echo -e "${YELLOW}Cleaning build artifacts...${NC}"
    cd "$DEBUG_DIR"
    make clean
    rm -f "$BIN_FILE"
    echo -e "${GREEN}✓ Clean complete${NC}"
    exit 0
fi

# Build
echo -e "${YELLOW}Building...${NC}"
cd "$DEBUG_DIR"
make clean && make -j8 all

# Verify binary exists
if [[ ! -f "$BINARY" ]]; then
    echo -e "${RED}✗ Build failed: $BINARY not found${NC}"
    exit 1
fi

# Get binary size
BINARY_SIZE=$(ls -lh "$BINARY" | awk '{print $5}')
echo -e "${GREEN}✓ Build successful${NC}"
echo -e "  ELF: $BINARY ($BINARY_SIZE)"

# Generate bin file for flashing
arm-none-eabi-objcopy -O binary "$BINARY" "$BIN_FILE"
BIN_SIZE=$(ls -lh "$BIN_FILE" | awk '{print $5}')
echo -e "  BIN: $BIN_FILE ($BIN_SIZE)"

echo -e "${YELLOW}========================================${NC}"
echo -e "${GREEN}Ready to flash or debug${NC}"
echo -e "${YELLOW}========================================${NC}"
