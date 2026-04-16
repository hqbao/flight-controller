# Importing to STM32CubeIDE After Git Clone

This guide walks through importing the flight-controller project into STM32CubeIDE on a fresh clone.

## Prerequisites

- **STM32CubeIDE** 1.13+ installed
- Repository cloned:
  ```
  flight-controller/
  ├── base/
  │   ├── boards/h7v1/              ← CubeIDE project lives here
  │   └── foundation/               ← linked into project
  ├── libs/robotkit/                ← pre-built static library (included)
  └── modules/                      ← linked into project
  ```

## Step 1: Import the Project

1. Open STM32CubeIDE
2. **File → Import → General → Existing Projects into Workspace**
3. Set **Root Directory** to:
   ```
   <your-clone>/flight-controller/base/boards/h7v1
   ```
4. The project **h7v1** should appear in the list. Make sure it is checked.
5. **Do NOT check** "Copy projects into workspace" — the project must stay in place so linked resources resolve correctly.
6. Click **Finish**

## Step 2: Verify Linked Resources

The `.project` file defines two linked folders that point outside the project directory:

| Virtual Folder | Resolves To |
|----------------|-------------|
| `foundation` | `flight-controller/base/foundation/` |
| `modules` | `flight-controller/modules/` |

These use the `PARENT-N-PROJECT_LOC` variable (relative to the project root), so they resolve automatically — no manual path configuration needed.

**To verify:** In the Project Explorer, expand the project. You should see `foundation/` and `modules/` folders with a small arrow icon indicating they are linked resources. If they show errors, right-click the project → **Properties → Resource → Linked Resources** and confirm the paths resolve.

## Step 3: Verify Build Configuration

The project is pre-configured with:

| Setting | Value |
|---------|-------|
| MCU | STM32H743VITx |
| Toolchain | GNU ARM Embedded (arm-none-eabi-gcc) |
| FPU | FPv5-D16 (Hard ABI) |
| Linker Script | `STM32H743VITX_FLASH.ld` |
| Library | `librobotkit-stm32h7.a` (in `libs/robotkit/`) |

Key include paths (already set in `.cproject`):
```
../Core/Inc
../Drivers/STM32H7xx_HAL_Driver/Inc
../Drivers/CMSIS/Device/ST/STM32H7xx/Include
../Drivers/CMSIS/Include
../../../../libs/robotkit
../../../../modules
../../../foundation
../platform
```

## Step 4: Build

### Option A: Build from STM32CubeIDE

1. Select the **Debug** build configuration (Project → Build Configurations → Set Active → Debug)
2. **Project → Build Project** (or Ctrl+B / Cmd+B)
3. Output: `Debug/h7v1.elf` and `Debug/h7v1.bin`

### Option B: Build from Terminal (Recommended)

The `build.sh` script applies necessary patches that CubeIDE's managed build may overwrite:

```bash
cd flight-controller/base/boards/h7v1
./build.sh
```

This script:
- Patches CubeIDE-generated makefiles to include the `platform/` folder
- Removes stale references CubeIDE may regenerate
- Adds modules that CubeIDE doesn't track (fft, calibration, config, dblink)
- Strips `-fcyclomatic-complexity` flag on macOS (incompatible with Homebrew toolchain)

> **Important:** After regenerating code with CubeMX (modifying `h7v1.ioc`), always run `./build.sh` to re-apply patches. CubeIDE regenerates the `Debug/` makefiles and may drop custom additions.

## Step 5: Flash & Debug

### From Terminal
```bash
./build-flash.sh              # Build + flash via ST-Link
./build-flash.sh --verify     # Build + flash + verify
./build-flash.sh --debug      # Build + flash + start GDB
```

### From STM32CubeIDE
1. Connect ST-Link V2 to the board
2. **Run → Debug As → STM32 C/C++ Application**
3. Use the existing `h7v1 Debug.launch` configuration

## Troubleshooting

### "Unresolved inclusion" errors in the editor
The CubeIDE indexer may not find headers in linked folders. This does not affect the build. To fix indexer warnings:
1. Right-click project → **Properties → C/C++ General → Paths and Symbols**
2. Verify the include paths listed in Step 3 are present under **GNU C**
3. Right-click project → **Index → Rebuild**

### Build fails after CubeMX code regeneration
CubeMX overwrites `Debug/` makefiles and may remove custom `platform/` and module entries. Run `./build.sh` from the terminal — it re-applies all necessary patches automatically.

### "Program file does not exist" when debugging
Make sure the active build configuration is **Debug** (not Release). The launch configuration expects `Debug/h7v1.elf`.

### Linked folders show as empty or missing
The linked resources use relative paths. This happens if:
- The project was copied out of the repository structure
- "Copy projects into workspace" was checked during import

Fix: re-import without copying, keeping the project inside the git clone.

### macOS: arm-none-eabi-gcc not found
Install the ARM toolchain via Homebrew:
```bash
brew install arm-none-eabi-gcc
```
The `build.sh` script automatically prioritizes the Homebrew toolchain over CubeIDE's bundled version.
