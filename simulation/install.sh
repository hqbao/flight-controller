#!/bin/bash
# install.sh - Setup development environment for Skydev Flight Controller (SITL & Firmware)
# Supports macOS (Homebrew) and Linux (apt)

set -e

# Increase file open limit to avoid Homebrew errors
if [[ "$(uname -s)" == "Darwin" ]]; then
    ulimit -n 10240 || true
fi

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo_info() { echo -e "${BLUE}ℹ️  $1${NC}"; }
echo_ok() { echo -e "${GREEN}✅ $1${NC}"; }
echo_err() { echo -e "${RED}❌ $1${NC}"; }

# Detect OS
OS="$(uname -s)"
echo_info "Detected OS: $OS"

if [[ "$OS" == "Darwin" ]]; then
    # --- macOS Setup ---

    # 1. Check Homebrew
    if ! command -v brew &> /dev/null; then
        echo_info "Homebrew not found. Installing..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    else
        echo_ok "Homebrew is installed"
    fi

    # 2. Install/Check Basic Tools
    TOOLS=(cmake ninja wget git python3)
    echo_info "Checking System Tools..."
    for tool in "${TOOLS[@]}"; do
        if ! brew list $tool &> /dev/null; then
            echo_info "Installing $tool..."
            brew install $tool
        else
            echo_ok "$tool is installed"
        fi
    done

    # 3. Install SDL2 (Joystick support)
    echo_info "Checking SDL2..."
    if ! brew list sdl2 &> /dev/null; then
        echo_info "Installing sdl2..."
        brew install sdl2
    else
        echo_ok "sdl2 is installed"
    fi

    # 4. Install Gazebo Harmonic
    echo_info "Checking Gazebo Harmonic..."
    if ! brew tap | grep -q "osrf/simulation"; then
        echo_info "Tapping osrf/simulation..."
        brew tap osrf/simulation
    fi

    if ! brew list gz-harmonic &> /dev/null; then
        echo_info "Installing gz-harmonic..."
        brew install gz-harmonic
    else
        echo_ok "gz-harmonic is installed"
    fi

elif [[ "$OS" == "Linux" ]]; then
    # --- Linux Setup (Ubuntu/Debian) ---
    echo_info "Running apt-get update..."
    sudo apt-get update

    echo_info "Installing dependencies..."
    sudo apt-get install -y build-essential cmake git python3 python3-pip libsdl2-dev
    
    # Gazebo install (simplified, assumes user knows how to setup gz repo or uses standard)
    # See https://gazebosim.org/docs/harmonic/install_ubuntu
    echo_info "Note: For Gazebo Harmonic on Linux, please follow official instructions at gazebosim.org"

else
    echo_err "Unsupported OS: $OS"
    exit 1
fi

# 5. Python Dependencies
echo_info "Checking Python dependencies..."
# protobuf is needed for Gazebo messages
# Note: specific pip upgrade removed to avoid conflict with Homebrew managed pip
pip3 install protobuf grpcio

# 6. Check Gazebo Python Bindings (macOS Fix)
if [[ "$OS" == "Darwin" ]]; then
    echo_info "Verifying Gazebo Python bindings..."
    # Find brew prefix
    BREW_PREFIX=$(brew --prefix)
    # Common paths for python bindings
    POSSIBLE_PATHS=(
        "$BREW_PREFIX/lib/python3.13/site-packages"
        "$BREW_PREFIX/lib/python3.12/site-packages"
        "$BREW_PREFIX/lib/python3.11/site-packages"
        "$BREW_PREFIX/opt/gz-transport13/lib/python3.11/site-packages"
    )

    FOUND_GZ=false
    for path in "${POSSIBLE_PATHS[@]}"; do
        if [ -d "$path/gz" ]; then
            echo_info "Found gz bindings at: $path"
            export PYTHONPATH=$PYTHONPATH:$path
            FOUND_GZ=true
            break
        fi
    done
    
    if [ "$FOUND_GZ" = false ]; then
        echo_info "⚠️  Could not automatically locate 'gz' python package. You might need to set PYTHONPATH manually."
    fi
fi

# 7. Build Flight Controller (SITL)
echo_info "Building Flight Controller for SITL..."
# Navigate from flight-controller/simulation/ to flight-controller/
FC_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [ -d "$FC_ROOT/base/boards/sitl_macos" ]; then
    (cd "$FC_ROOT/base/boards/sitl_macos" && make)
    if [ $? -eq 0 ]; then
        echo_ok "Flight Controller built successfully"
    else
        echo_err "Failed to build Flight Controller"
    fi
else
    echo_err "SITL Board directory not found at $FC_ROOT/base/boards/sitl_macos"
fi

echo_ok "Environment setup complete!"
echo_info "You can now run the simulation:"
echo "   ./flight-controller/simulation/run_sitl_macos.sh"
