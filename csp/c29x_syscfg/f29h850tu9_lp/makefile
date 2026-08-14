# Standalone validation build for the CCS projectspec sources.
# Override these variables when TI tools are installed elsewhere.
C29SDK_ROOT ?= C:/ti/f29h85x-sdk_1_02_01_00
CCS_PATH ?= C:/ti/ccs2100/ccs
CGT29X_PATH ?= $(CCS_PATH)/tools/compiler/ti-cgt-c29_2.2.0.LTS
SYSCFG ?= C:/ti/sysconfig_1.25.0/sysconfig_cli.bat
GMP_PRO_LOCATION ?= ../../..

COMPILER := $(CGT29X_PATH)/bin/c29clang.exe
TARGET := f29h850tu9_gmp_blinky
BUILD := build
SYSCFG_OUT := $(BUILD)/syscfg
OBJ := $(BUILD)/obj

INCLUDES := \
	-I"$(GMP_PRO_LOCATION)" \
	-I"$(GMP_PRO_LOCATION)/csp/c29x_syscfg" \
	-I"xplt" -I"user" -I"$(SYSCFG_OUT)" \
	-I"$(C29SDK_ROOT)/examples/device_support/include" \
	-I"$(C29SDK_ROOT)/source" \
	-I"$(C29SDK_ROOT)/source/security" \
	-I"$(C29SDK_ROOT)/source/driverlib" \
	-I"$(C29SDK_ROOT)/source/bitfields" \
	-I"$(C29SDK_ROOT)/source/kernel/dpl" \
	-I"$(CGT29X_PATH)/include/c"

CFLAGS := -g -O1 -DDEBUG -D_LAUNCHXL_F29H85X -Qunused-arguments $(INCLUDES)
LDFLAGS := -Wl,--entry_point="code_start",--stack_size=0x2000,--heap_size=0x800,--rom_model,--reread_libs,--diag_suppress=10325-D,--diag_suppress=10063-D
LIB_PATHS := \
	-Wl,-i,"$(CGT29X_PATH)/lib" \
	-Wl,-i,"$(C29SDK_ROOT)/source/driverlib/ccs" \
	-Wl,-i,"$(C29SDK_ROOT)/source/kernel/nortos/ccs"
LIBS := -ldriverlib.lib -ldpl_nortos.lib

OBJECTS := \
	$(OBJ)/user_main.o \
	$(OBJ)/gmp_std_port.o \
	$(OBJ)/gmp_datalink.o \
	$(OBJ)/sysconfig_adapter.o \
	$(OBJ)/peripheral_driver.o \
	$(OBJ)/board.o \
	$(OBJ)/device.o \
	$(OBJ)/ti_drivers_config.o \
	$(OBJ)/ti_dpl_config.o \
	$(OBJ)/bitfield_structs.o \
	$(OBJ)/codestartbranch.o

.PHONY: all clean syscfg

all: $(BUILD)/$(TARGET).out

syscfg: $(SYSCFG_OUT)/board.c

$(SYSCFG_OUT)/board.c: f29h850tu9.syscfg
	@if not exist "$(SYSCFG_OUT)" mkdir "$(SYSCFG_OUT)"
	"$(SYSCFG)" --script "$<" -o "$(SYSCFG_OUT)" -s "$(C29SDK_ROOT)/.metadata/sdk.json" --compiler ticlang

$(SYSCFG_OUT)/device.c $(SYSCFG_OUT)/ti_drivers_config.c $(SYSCFG_OUT)/ti_dpl_config.c: $(SYSCFG_OUT)/board.c

$(OBJ):
	@if not exist "$(OBJ)" mkdir "$(OBJ)"

$(OBJ)/user_main.o: user/user_main.c $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/gmp_std_port.o: $(GMP_PRO_LOCATION)/core/rt/src/gmp_runtime.c $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/gmp_datalink.o: $(GMP_PRO_LOCATION)/core/dev/datalink/src/gmp_datalink.c $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/sysconfig_adapter.o: $(GMP_PRO_LOCATION)/csp/c29x_syscfg/src/sysconfig_adapter.c $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/peripheral_driver.o: $(GMP_PRO_LOCATION)/csp/c29x_syscfg/src/peripheral_driver.c $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/board.o: $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/device.o: $(SYSCFG_OUT)/device.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/ti_drivers_config.o: $(SYSCFG_OUT)/ti_drivers_config.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/ti_dpl_config.o: $(SYSCFG_OUT)/ti_dpl_config.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/bitfield_structs.o: $(C29SDK_ROOT)/source/bitfields/bitfield_structs.c $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(OBJ)/codestartbranch.o: $(C29SDK_ROOT)/examples/device_support/source/codestartbranch.asm | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(BUILD)/$(TARGET).out: $(OBJECTS)
	"$(COMPILER)" $(INCLUDES) $(LDFLAGS) $(LIB_PATHS) -Wl,-m="$(BUILD)/$(TARGET).map" -o "$@" $(OBJECTS) $(LIBS) "$(C29SDK_ROOT)/examples/device_support/cmd/f29h85x_ram_lnk.cmd"

clean:
	@echo Remove the build directory to clean this project.
