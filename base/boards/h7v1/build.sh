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
    sed -i '' '/^modules\/flight_state/a\
platform \\
' "$DEBUG_DIR/sources.mk"
fi

# Add platform .o files to objects.list (if missing)
if ! grep -q 'platform/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./platform/dshot.o"\n"./platform/dshot_ex.o"\n"./platform/module.o"\n"./platform/platform_common.o"\n"./platform/platform_i2c.o"\n"./platform/platform_pwm.o"\n"./platform/platform_rc.o"\n"./platform/platform_spi.o"\n"./platform/platform_uart.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Remove dshot from Core/Src in objects.list (they live in platform/ now)
sed -i '' '/\.\/Core\/Src\/dshot\.o/d;/\.\/Core\/Src\/dshot_ex\.o/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true

# Remove dshot_bidir references (source file doesn't exist yet)
sed -i '' '/dshot_bidir/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
sed -i '' '/dshot_bidir/d' "$DEBUG_DIR/platform/subdir.mk" 2>/dev/null || true

# Move module.c from foundation/ to platform/ (module.c is board-specific, not shared)
# CubeIDE may regenerate it under foundation/ — always fix it
sed -i '' '/foundation\/module/d' "$DEBUG_DIR/foundation/subdir.mk" 2>/dev/null || true
sed -i '' '/foundation\/module/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true

# Remove platform.c from foundation/ (wrappers eliminated — boards call publish() directly)
sed -i '' '/foundation\/platform[^_]/d' "$DEBUG_DIR/foundation/subdir.mk" 2>/dev/null || true
sed -i '' '/foundation\/platform[^_]/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
# After deleting platform.o target line, its recipe (tab-indented gcc) becomes orphaned.
# Delete only the first orphaned tab-gcc line (one-shot, preserves pubsub.o recipe).
awk '!deleted && /^\t.*arm-none-eabi-gcc/ && !prev_is_target {deleted=1; next} {prev_is_target = /^[a-z].*:/; print}' \
    "$DEBUG_DIR/foundation/subdir.mk" > "$DEBUG_DIR/foundation/subdir.mk.tmp" && \
    mv "$DEBUG_DIR/foundation/subdir.mk.tmp" "$DEBUG_DIR/foundation/subdir.mk"
if ! grep -q 'platform/module' "$DEBUG_DIR/platform/subdir.mk" 2>/dev/null; then
    sed -i '' 's|../platform/dshot_ex.c \\|../platform/dshot_ex.c \\\
../platform/module.c \\|' "$DEBUG_DIR/platform/subdir.mk"
    sed -i '' 's|./platform/dshot_ex.o \\|./platform/dshot_ex.o \\\
./platform/module.o \\|' "$DEBUG_DIR/platform/subdir.mk"
    sed -i '' 's|./platform/dshot_ex.d \\|./platform/dshot_ex.d \\\
./platform/module.d \\|' "$DEBUG_DIR/platform/subdir.mk"
fi
if ! grep -q 'platform/module' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./platform/module.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Remove deleted modules (oscillation_detection, linear_drift_detection) from CubeIDE build
sed -i '' '/oscillation_detection/d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/linear_drift_detection/d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/oscillation_detection/d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/linear_drift_detection/d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/oscillation_detection/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
sed -i '' '/linear_drift_detection/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
rm -rf "$DEBUG_DIR/modules/oscillation_detection" "$DEBUG_DIR/modules/linear_drift_detection" 2>/dev/null || true

# Remove logger module (merged into db_sender)
sed -i '' '/modules\/logger/d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/modules\/logger/d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/modules\/logger/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true

# Migrate db_reader + db_sender → dblink (consolidated module)
sed -i '' '/modules\/db_reader\//d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/modules\/db_sender\//d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/^modules\/db_reader /d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/^modules\/db_sender /d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/\/db_reader\/db_reader/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
sed -i '' '/\/db_sender\/db_sender/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
if ! grep -q 'modules/dblink' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/config/subdir.mk|-include modules/dblink/subdir.mk\
-include modules/config/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/dblink' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/config/a\
modules/dblink \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/dblink/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/dblink/dblink.o"\n' >> "$DEBUG_DIR/objects.list"
    printf '"./modules/dblink/db_reader.o"\n' >> "$DEBUG_DIR/objects.list"
    printf '"./modules/dblink/db_sender.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Add fft module to build (if missing — CubeIDE doesn't know about it)
if ! grep -q 'modules/fft' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/fault_handler/subdir.mk|-include modules/fft/subdir.mk\
-include modules/fault_handler/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/fft' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/fault_handler/a\
modules/fft \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/fft/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/fft/fft.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Add calibration module to build (if missing — CubeIDE doesn't know about it)
if ! grep -q 'modules/calibration' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/compass/subdir.mk|-include modules/calibration/subdir.mk\
-include modules/compass/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/calibration' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/compass/a\
modules/calibration \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/calibration/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/calibration/calibration.o"\n' >> "$DEBUG_DIR/objects.list"
    printf '"./modules/calibration/calibration_gyro.o"\n' >> "$DEBUG_DIR/objects.list"
    printf '"./modules/calibration/calibration_accel.o"\n' >> "$DEBUG_DIR/objects.list"
    printf '"./modules/calibration/calibration_mag.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Add config module to build (if missing — CubeIDE doesn't know about it)
if ! grep -q 'modules/config' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/calibration/subdir.mk|-include modules/config/subdir.mk\
-include modules/calibration/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/config' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/calibration/a\
modules/config \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/config/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/config/config.o"\n' >> "$DEBUG_DIR/objects.list"
fi
# Create config subdir.mk (build rules for config module)
CONFIG_DIR="$DEBUG_DIR/modules/config"
mkdir -p "$CONFIG_DIR"
if [[ ! -f "$CONFIG_DIR/subdir.mk" ]]; then
    MODULES_ABS="$(cd "$PROJECT_DIR/../../../modules" && pwd)"
    cat > "$CONFIG_DIR/subdir.mk" << SUBMK
C_SRCS += \\
$MODULES_ABS/config/config.c

OBJS += \\
./modules/config/config.o

C_DEPS += \\
./modules/config/config.d

modules/config/config.o: $MODULES_ABS/config/config.c modules/config/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

clean: clean-modules-2f-config

clean-modules-2f-config:
	-\$(RM) ./modules/config/config.cyclo ./modules/config/config.d ./modules/config/config.o ./modules/config/config.su

.PHONY: clean-modules-2f-config
SUBMK
fi

# Add mix_control module to build (if missing — CubeIDE doesn't know about it)
if ! grep -q 'modules/mix_control' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/attitude_control/subdir.mk|-include modules/mix_control/subdir.mk\
-include modules/attitude_control/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/mix_control' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/attitude_control/a\
modules/mix_control \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/mix_control/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/mix_control/mix_control.o"\n' >> "$DEBUG_DIR/objects.list"
fi
if ! grep -q 'mix_control/quadcopter.o' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/mix_control/quadcopter.o"\n' >> "$DEBUG_DIR/objects.list"
fi
if ! grep -q 'mix_control/bicopter.o' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/mix_control/bicopter.o"\n' >> "$DEBUG_DIR/objects.list"
fi
# Create mix_control subdir.mk (build rules for mix_control module)
MIXCTL_DIR="$DEBUG_DIR/modules/mix_control"
mkdir -p "$MIXCTL_DIR"
MODULES_ABS_MIX="$(cd "$PROJECT_DIR/../../../modules" && pwd)"
cat > "$MIXCTL_DIR/subdir.mk" << SUBMK
C_SRCS += \\
$MODULES_ABS_MIX/mix_control/mix_control.c \\
$MODULES_ABS_MIX/mix_control/quadcopter.c \\
$MODULES_ABS_MIX/mix_control/bicopter.c

OBJS += \\
./modules/mix_control/mix_control.o \\
./modules/mix_control/quadcopter.o \\
./modules/mix_control/bicopter.o

C_DEPS += \\
./modules/mix_control/mix_control.d \\
./modules/mix_control/quadcopter.d \\
./modules/mix_control/bicopter.d

modules/mix_control/mix_control.o: $MODULES_ABS_MIX/mix_control/mix_control.c modules/mix_control/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -I../../../../modules/mix_control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

modules/mix_control/quadcopter.o: $MODULES_ABS_MIX/mix_control/quadcopter.c modules/mix_control/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -I../../../../modules/mix_control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

modules/mix_control/bicopter.o: $MODULES_ABS_MIX/mix_control/bicopter.c modules/mix_control/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -I../../../../modules/mix_control -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

clean: clean-modules-2f-mix_control

clean-modules-2f-mix_control:
	-\$(RM) ./modules/mix_control/mix_control.cyclo ./modules/mix_control/mix_control.d ./modules/mix_control/mix_control.o ./modules/mix_control/mix_control.su ./modules/mix_control/quadcopter.cyclo ./modules/mix_control/quadcopter.d ./modules/mix_control/quadcopter.o ./modules/mix_control/quadcopter.su ./modules/mix_control/bicopter.cyclo ./modules/mix_control/bicopter.d ./modules/mix_control/bicopter.o ./modules/mix_control/bicopter.su

.PHONY: clean-modules-2f-mix_control
SUBMK

# Add flight_telemetry module to build (if missing — CubeIDE doesn't know about it)
if ! grep -q 'modules/flight_telemetry' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/flight_state/subdir.mk|-include modules/flight_telemetry/subdir.mk\
-include modules/flight_state/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/flight_telemetry' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/flight_state/a\
modules/flight_telemetry \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/flight_telemetry/' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/flight_telemetry/flight_telemetry.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Add notch_filter module to build (if missing)
# Clean up old naming (notch_filter_module -> notch_filter)
sed -i '' '/notch_filter_module/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
if ! grep -q 'modules/notch_filter' "$DEBUG_DIR/makefile" 2>/dev/null; then
    sed -i '' 's|-include modules/mix_control/subdir.mk|-include modules/notch_filter/subdir.mk\
-include modules/mix_control/subdir.mk|' "$DEBUG_DIR/makefile"
fi
if ! grep -q 'modules/notch_filter' "$DEBUG_DIR/sources.mk" 2>/dev/null; then
    sed -i '' '/^modules\/mix_control/a\
modules/notch_filter \\
' "$DEBUG_DIR/sources.mk"
fi
if ! grep -q 'modules/notch_filter/notch_filter.o' "$DEBUG_DIR/objects.list" 2>/dev/null; then
    printf '"./modules/notch_filter/notch_filter.o"\n' >> "$DEBUG_DIR/objects.list"
fi

# Clean up old module names (db_parser, uart_tx, db_reader, db_sender → all now in dblink)
sed -i '' '/modules\/db_parser/d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/modules\/db_parser/d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/modules\/db_parser/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true
sed -i '' '/modules\/uart_tx/d' "$DEBUG_DIR/makefile" 2>/dev/null || true
sed -i '' '/modules\/uart_tx/d' "$DEBUG_DIR/sources.mk" 2>/dev/null || true
sed -i '' '/modules\/uart_tx/d' "$DEBUG_DIR/objects.list" 2>/dev/null || true

# Create dblink subdir.mk (if missing)
DBLINK_DIR="$DEBUG_DIR/modules/dblink"
mkdir -p "$DBLINK_DIR"
if [[ ! -f "$DBLINK_DIR/subdir.mk" ]]; then
    MODULES_ABS="$(cd "$PROJECT_DIR/../../../modules" && pwd)"
    cat > "$DBLINK_DIR/subdir.mk" << SUBMK
C_SRCS += \\
$MODULES_ABS/dblink/dblink.c \\
$MODULES_ABS/dblink/db_reader.c \\
$MODULES_ABS/dblink/db_sender.c

OBJS += \\
./modules/dblink/dblink.o \\
./modules/dblink/db_reader.o \\
./modules/dblink/db_sender.o

C_DEPS += \\
./modules/dblink/dblink.d \\
./modules/dblink/db_reader.d \\
./modules/dblink/db_sender.d

modules/dblink/dblink.o: $MODULES_ABS/dblink/dblink.c modules/dblink/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

modules/dblink/db_reader.o: $MODULES_ABS/dblink/db_reader.c modules/dblink/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

modules/dblink/db_sender.o: $MODULES_ABS/dblink/db_sender.c modules/dblink/subdir.mk
	arm-none-eabi-gcc "\$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../../../../libs/robotkit -I../../../../modules -I../../../foundation -I../platform -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"\$(@:%.o=%.d)" -MT"\$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "\$@"

clean: clean-modules-2f-dblink

clean-modules-2f-dblink:
	-\$(RM) ./modules/dblink/dblink.cyclo ./modules/dblink/dblink.d ./modules/dblink/dblink.o ./modules/dblink/dblink.su ./modules/dblink/db_reader.cyclo ./modules/dblink/db_reader.d ./modules/dblink/db_reader.o ./modules/dblink/db_reader.su ./modules/dblink/db_sender.cyclo ./modules/dblink/db_sender.d ./modules/dblink/db_sender.o ./modules/dblink/db_sender.su

.PHONY: clean-modules-2f-dblink
SUBMK
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
