#!/bin/bash
# Skydev Flight Controller - Initial Setup (macOS)
# Run once after cloning the repository
# Usage: ./setup.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  Skydev Flight Controller - Setup      ║${NC}"
echo -e "${BLUE}║  macOS / Linux / WSL2 Edition          ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# Verify OS and install tools
OS="$(uname -s)"
case "$OS" in
    Darwin*)
        echo -e "${YELLOW}[1/4] Detecting OS: macOS${NC}"
        # Check Homebrew
        if ! command -v brew &> /dev/null; then
            echo -e "${RED}✗ Homebrew not installed${NC}"
            echo -e "${YELLOW}Install from: https://brew.sh${NC}"
            exit 1
        fi
        
        echo -e "${YELLOW}[2/4] Installing toolchain (Homebrew)...${NC}"
        brew install arm-none-eabi-gcc stlink openocd
        ;;
    Linux*)
        echo -e "${YELLOW}[1/4] Detecting OS: Linux${NC}"
        echo -e "${YELLOW}[2/4] Installing toolchain (apt-get)...${NC}"
        
        # Check if running as root or has sudo
        if [ "$EUID" -ne 0 ] && ! command -v sudo &> /dev/null; then
            echo -e "${RED}✗ sudo is required to install packages${NC}"
            exit 1
        fi

        CMD_PREFIX=""
        if [ "$EUID" -ne 0 ]; then
            CMD_PREFIX="sudo"
        fi

        $CMD_PREFIX apt-get update
        $CMD_PREFIX apt-get install -y gcc-arm-none-eabi openocd build-essential
        
        # Check for stlink-tools
        if apt-cache search stlink-tools | grep -q stlink-tools; then
            $CMD_PREFIX apt-get install -y stlink-tools
        else
            echo -e "${YELLOW}Note: stlink-tools package not found in default repos, trying stlink...${NC}"
             $CMD_PREFIX apt-get install -y stlink || echo -e "${YELLOW}Warning: Automatic stlink installation failed. Check your distro's repos.${NC}"
        fi
        ;;
    *)
        echo -e "${RED}✗ Unsupported OS: $OS${NC}"
        echo -e "${YELLOW}See SETUP_WSL2.md for manual instructions${NC}"
        exit 1
        ;;
esac

echo -e "${GREEN}✓ Toolchain installed${NC}"
echo ""

echo ""

# Configure project
echo -e "${YELLOW}[3/4] Configuring project...${NC}"

STM32_PROJECT="$SCRIPT_DIR"

if [[ ! -d "$STM32_PROJECT/Debug" ]]; then
    echo -e "${RED}✗ STM32 Debug directory not found${NC}"
    echo -e "${YELLOW}   Open h7v1.ioc in STM32CubeIDE and generate code${NC}"
    exit 1
fi

chmod +x "$STM32_PROJECT/build.sh" 2>/dev/null || true
chmod +x "$STM32_PROJECT/build-flash.sh" 2>/dev/null || true

echo -e "${GREEN}✓ Project configured${NC}"
echo ""

# Verify structure
echo -e "${YELLOW}[4/4] Verifying structure...${NC}"

REQUIRED_DIRS=(
    "flight-controller"
    "flight-controller/base/boards/h7v1"
    "flight-controller/modules"
    "flight-controller/libs"
    "robotkit"
    "optflow"
)

for dir in "${REQUIRED_DIRS[@]}"; do
    if [[ ! -d "$PROJECT_ROOT/$dir" ]]; then
        echo -e "${RED}✗ Missing: $dir${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✓ All directories present${NC}"
echo ""

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║  ✓ Setup Complete!                    ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Next steps:${NC}"
echo -e "  ${GREEN}1. Build:${NC}  cd $STM32_PROJECT && ./build.sh"
echo -e "  ${GREEN}2. Flash:${NC}  ./build-flash.sh"
echo -e "  ${GREEN}3. Debug:${NC}  ./build-flash.sh --debug"
echo ""
