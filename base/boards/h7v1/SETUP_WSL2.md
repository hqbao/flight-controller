# STM32H7 Flight Controller - WSL2 Setup

## Using WSL2 (Windows Subsystem for Linux)

Windows Subsystem for Linux 2 gives you a Linux environment inside Windows, making development much easier.

### Step 1: Install WSL2 & USBIPD

**1. Install WSL2:**
Run in PowerShell (Admin):
```powershell
wsl --install
```

**2. Install USBIPD (Required for USB access):**
WSL2 cannot access USB devices by default. You MUST install `usbipd-win` on Windows:
- Download & Install: [usbipd-win releases](https://github.com/dorssel/usbipd-win/releases)

Restart your computer after installation.

### Step 2: Clone Project in WSL2

Open Ubuntu (WSL2) terminal:

```bash
cd /mnt/c/Users/YourUsername/Documents
git clone <your-repo>
cd skydev/flight-controller/base/boards/h7v1
```

### Step 3: Install Tools

Run the automated setup script:

```bash
./setup.sh
```

Or install manually:

```bash
# Install build toolchain
sudo apt-get update
sudo apt-get install -y gcc-arm-none-eabi stlink-tools openocd build-essential
```

### Step 4: Connect ST-Link (Critical)

1. Plug in your ST-Link to USB
2. Open **Windows PowerShell (Admin)** and list devices:
   ```powershell
   usbipd list
   ```
3. Attach the device to WSL (replace `<BUSID>` with your ST-Link ID):
   ```powershell
   usbipd wsl attach --busid <BUSID>
   ```

### Step 5: Debugging

**Option 1: Command Line**
```bash
./build-flash.sh --debug
```

**Option 2: VS Code (Recommended)**
1. Open path in VS Code: `code .`
2. Go to **Run and Debug** tab
3. Select **STM32H7: Attach (GDB)** and press F5

Note: VS Code must be running in Remote-WSL mode (green icon in bottom-left).

---

## Why WSL2?

- ✅ Same build/flash scripts as macOS
- ✅ USB support via usbipd-win
- ✅ No compatibility issues
- ✅ Easy to use and fast
- ✅ Full Linux environment with apt package manager

---

**Note**: Native Windows is not supported. Use WSL2 for Windows development.
