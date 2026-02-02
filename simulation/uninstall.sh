#!/bin/bash
# uninstall_simulation.sh - Removes Skydev Simulation components
# CLEANUP ONLY - Does not remove standard dev tools like Python, Git, CMake.

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo_info() { echo -e "${BLUE}ℹ️  $1${NC}"; }
echo_ok() { echo -e "${GREEN}✅ $1${NC}"; }
echo_warn() { echo -e "${RED}⚠️  $1${NC}"; }

# Detect OS
OS="$(uname -s)"

echo_info "Starting Simulation Cleanup..."

# 1. Cleanup Compiled Binaries
echo_info "Removing compiled Flight Controller binary..."
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
FC_PATH="$SCRIPT_DIR/../base/boards/sitl_macos"

if [ -d "$FC_PATH" ]; then
    rm -f "$FC_PATH/fly_sitl"
    rm -f "$FC_PATH"/*.o
    echo_ok "Binaries removed."
else
    echo_warn "Flight controller directory not found at $FC_PATH"
fi

# 2. Cleanup System Dependencies (macOS/Linux)
if [[ "$OS" == "Darwin" ]]; then
    echo_info "Checking Gaszebo/Ignition packages to remove..."
    
    # 1. Aggressive Gazebo Cleanup
    # Expanded list including dependencies that might block SDL2 removal or Untap
    GAZEBO_DEPS=(
        gz-harmonic gz-cmake3 gz-common5 gz-fuel-tools9 gz-gui8 gz-math7 gz-msgs10 
        gz-physics7 gz-plugin2 gz-rendering8 gz-sensors8 gz-sim8 gz-tools2 
        gz-transport13 gz-utils2 
        ignition-cmake2 ignition-common3 ignition-math6 
        sdformat14 ogre1.9 ogre2.3 dartsim open-scene-graph
    )
    
    FOUND_GZ=false
    for pkg in "${GAZEBO_DEPS[@]}"; do
        if brew list "$pkg" &> /dev/null; then
            FOUND_GZ=true
            break
        fi
    done

    if [ "$FOUND_GZ" = true ] || brew list gz-harmonic &> /dev/null ; then
        echo_warn "Found Gazebo/Ignition simulation packages."
        echo_warn "Uninstall all Gazebo-related packages? (y/n)"
        read -r gz_resp
        if [[ "$gz_resp" =~ ^([yY][eE][sS]|[yY])$ ]]; then
             for pkg in "${GAZEBO_DEPS[@]}"; do
                if brew list "$pkg" &> /dev/null; then
                    echo "Removing $pkg..."
                    # Ignore dependencies here to ensure we strip them out even if they depend on each other cyclically
                    brew uninstall --ignore-dependencies "$pkg" || echo "  - Failed to remove $pkg"
                fi
             done
             
             # Attempt Untap
             if brew tap | grep -q "osrf/simulation"; then
                 echo_info "Untapping osrf/simulation..."
                 brew untap osrf/simulation || echo_warn "Could not untap osrf/simulation"
             fi
        fi
    else
        echo_info "No Gazebo packages found."
    fi

    # Remove SDL2 (Optional as it's a common lib)
    if brew list sdl2 &> /dev/null; then
        echo_warn "Uninstall sdl2? (y/n) [It is a common library, say no if unsure]"
        read -r sdl_resp
        if [[ "$sdl_resp" =~ ^([yY][eE][sS]|[yY])$ ]]; then
            echo_info "Attempting to uninstall sdl2..."
            if ! brew uninstall sdl2; then
                echo_warn "Standard uninstall failed." 
                echo_warn "This usually means valid apps (like ffmpeg) still need it."
                echo_warn "Force uninstall --ignore-dependencies? (y/n) [Warning: Might break ffmpeg]"
                read -r force_sdl
                if [[ "$force_sdl" =~ ^([yY][eE][sS]|[yY])$ ]]; then
                    brew uninstall --ignore-dependencies sdl2
                else
                    echo_info "Skipping forced uninstall of sdl2."
                fi
            fi
        else
            echo_info "Skipping sdl2."
        fi
    fi

elif [[ "$OS" == "Linux" ]]; then
    echo_info "Checking APT packages..."
    echo_warn "Uninstall gz-harmonic and libsdl2-dev? (y/n)"
    read -r lin_resp
    if [[ "$lin_resp" =~ ^([yY][eE][sS]|[yY])$ ]]; then
        sudo apt-get remove -y gz-harmonic libsdl2-dev
    fi
fi

echo_ok "Cleanup complete!"
