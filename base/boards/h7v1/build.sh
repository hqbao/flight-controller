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

# Pre-build: Patch CubeIDE-generated makefiles for platform/ folder integration
# STM32CubeIDE may regenerate Debug/ makefiles from .cproject, which resets our
# custom patches. This ensures the platform/ folder is always included.

# Strip -fcyclomatic-complexity (CubeIDE generates it, Homebrew toolchain rejects it)
find "$DEBUG_DIR" -name "subdir.mk" -exec sed -i '' 's/ -fcyclomatic-complexity//g' {} + 2>/dev/null || true

# Add -I../platform to Core/Src compile rules (if missing)
if ! grep -q '\-I\.\./platform' "$DEBUG_DIR/Core/Src/subdir.mk" 2>/dev/null; then
    sed -i '' 's|-I../../../foundation|-I../../../foundation -I../platform|g' "$DEBUG_DIR/Core/Src/subdir.mk"
fi

# Remove dshot references from Core/Src/subdir.mk (they live in platform/ now)
# CubeIDE may regenerate these entries — remove them to avoid duplicate symbols
sed -i '' '/\.\.\/Core\/Src\/dshot\.c/d;/\.\.\/Core\/Src\/dshot_ex\.c/d' "$DEBUG_DIR/Core/Src/subdir.mk" 2>/dev/null || true
sed -i '' '/\.\/Core\/Src\/dshot\.o/d;/\.\/Core\/Src\/dshot_ex\.o/d' "$DEBUG_DIR/Core/Src/subdir.mk" 2>/dev/null || true
sed -i '' '/\.\/Core\/Src\/dshot\.d/d;/\.\/Core\/Src\/dshot_ex\.d/d' "$DEBUG_DIR/Core/Src/subdir.mk" 2>/dev/null || true
sed -i '' 's| \./Core/Src/dshot\.cyclo \./Core/Src/dshot\.d \./Core/Src/dshot\.o \./Core/Src/dshot\.su \./Core/Src/dshot_ex\.cyclo \./Core/Src/dshot_ex\.d \./Core/Src/dshot_ex\.o \./Core/Src/dshot_ex\.su||g' "$DEBUG_DIR/Core/Src/subdir.mk" 2>/dev/null || true

# Add platform/subdir.mk to makefile (if missing)
if ! grep -q 'platform/subdir.mk' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include Core/Src/subdir.mk|-include platform/subdir.mk\
-include Core/Src/subdir.mk|' "$DEBUG_DIR/makefile"
fi

# Add platform to sources.mk SUBDIRS (if missing)
if ! grep -q '^platform' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/state_detector/a\
platform \\
' "$DEBUG_DIR/sources.mk"
fi

# Add platform .o files to objects.list (if missing)
if ! grep -q 'platform/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./platform/dshot.o"\n"./platform/dshot_ex.o"\n"./platform/platform_common.o"\n"./platform/platform_i2c.o"\n"./platform/platform_pwm.o"\n"./platform/platform_rc.o"\n"./platform/platform_spi.o"\n"./platform/platform_uart.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Remove dshot from Core/Src in objects.list (they live in platform/ now)
sed -i '' '/\.\/Core\/Src\/dshot\.o/d;/\.\/Core\/Src\/dshot_ex\.o/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true

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
