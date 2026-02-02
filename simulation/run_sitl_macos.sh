#!/bin/bash

# SITL Launcher for macOS (Split Server/GUI)
# Usage: ./simulation/run_sitl_macos.sh

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Stopping SITL..."
    pkill -9 -f "gz sim"
    pkill -9 -f "sitl_bridge.py"
    pkill -9 -f "fly_sitl"
    exit
}

# Trap Ctrl+C and Exit
trap cleanup SIGINT EXIT

# Get script directory for robust paths
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# Set IP for macOS to avoid multicast issues
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models"

echo "🧹 Cleaning up old processes..."
pkill -9 -f "gz sim"
pkill -9 -f "sitl_bridge.py"
pkill -9 -f "fly_sitl"
sleep 1

# 1. Start Gazebo Server (Physics)
echo "🌍 Starting Gazebo Server..."
# -s: Server only, -r: Run immediately (no pause)
gz sim -v3 -s -r "$SCRIPT_DIR/models/sitl_combined.sdf" > gz_server.log 2>&1 &
SERVER_PID=$!

echo "⏳ Waiting for server to initialize..."
sleep 5

# 2. Start SITL Bridge
echo "🌉 Starting Python Bridge..."
# Use explicit python path to ensure we get the environment with installed packages
PYTHON_EXEC="/opt/homebrew/Cellar/python@3.13/3.13.2/bin/python3"
if [ ! -f "$PYTHON_EXEC" ]; then
    PYTHON_EXEC="python3" # Fallback
fi
$PYTHON_EXEC -u "$SCRIPT_DIR/sitl_bridge.py" > bridge.log 2>&1 &
BRIDGE_PID=$!

# 3. Start Gazebo GUI
echo "📺 Starting Gazebo GUI..."
# -g: GUI only, stdout/stderr silenced to keep dashboard clean
gz sim -v3 -g > /dev/null 2>&1 &
GUI_PID=$!

if [ "$1" == "--no-fc" ]; then
    echo "⚡ Starting Environment Only (No Flight Controller)..."
    echo "   (Launch Flight Controller from VS Code Debugger now)"
    # Wait indefinitely so the script doesn't exit and kill child processes
    wait $GUI_PID
    exit
fi

# 4. Start Flight Controller
echo "🚁 Starting Flight Controller (Game Controller)..."
FC_PATH="$SCRIPT_DIR/../base/boards/sitl_macos/fly_sitl"

if [ -f "$FC_PATH" ]; then
    "$FC_PATH"
else
    echo "❌ Error: Flight Controller binary not found at $FC_PATH"
    echo "   Please run './flight-controller/simulation/install.sh' to build it."
    cleanup
fi
# Removing trailing '&' to let this run in foreground so user can see the output!
# FC_PID=$! # No PID tracking for foreground process

echo "✅ SITL Running. Press Ctrl+C to stop."
echo "   - Server PID: $SERVER_PID"
echo "   - Bridge PID: $BRIDGE_PID"
echo "   - GUI PID:    $GUI_PID"
# echo "   - FC PID:     $FC_PID"

# Wait for GUI to close or User interrupt
# wait $GUI_PID
