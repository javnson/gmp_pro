# Standalone C29 compiler validation for the PMSM target.
C29SDK_ROOT ?= C:/ti/f29h85x-sdk_1_02_01_00
CCS_PATH ?= C:/ti/ccs2100/ccs
CGT29X_PATH ?= $(CCS_PATH)/tools/compiler/ti-cgt-c29_2.2.0.LTS
SYSCFG ?= C:/ti/sysconfig_1.25.0/sysconfig_cli.bat
GMP_PRO_LOCATION ?= ../../../../..

COMPILER := $(CGT29X_PATH)/bin/c29clang.exe
BUILD := build
SYSCFG_OUT := $(BUILD)/syscfg
OBJ := $(BUILD)/obj
TARGET := mcs_pmsm_nt_f29h85x_3phgan

INCLUDES := \
	-I"$(GMP_PRO_LOCATION)" \
	-I"$(GMP_PRO_LOCATION)/csp/c29x_syscfg" \
	-I"$(GMP_PRO_LOCATION)/ctl/suite/mcs_pmsm_nt/src" \
	-I"." -I"xplt" -I"sdpe_mgr" -I"$(SYSCFG_OUT)" \
	-I"$(C29SDK_ROOT)/examples/device_support/include" \
	-I"$(C29SDK_ROOT)/source" \
	-I"$(C29SDK_ROOT)/source/security" \
	-I"$(C29SDK_ROOT)/source/driverlib" \
	-I"$(C29SDK_ROOT)/source/bitfields" \
	-I"$(C29SDK_ROOT)/source/kernel/dpl" \
	-I"$(CGT29X_PATH)/include/c"

CFLAGS := -g -O1 -DDEBUG -D_LAUNCHXL_F29H85X -Qunused-arguments $(INCLUDES)
LDFLAGS := -Wl,--entry_point="code_start",--stack_size=0x2000,--heap_size=0x800,--rom_model,--reread_libs,--diag_suppress=10325-D,--diag_suppress=10063-D
LIB_PATHS := -Wl,-i,"$(CGT29X_PATH)/lib" -Wl,-i,"$(C29SDK_ROOT)/source/driverlib/ccs" -Wl,-i,"$(C29SDK_ROOT)/source/kernel/nortos/ccs"
LIBS := -ldriverlib.lib -ldpl_nortos.lib

SOURCES := \
	$(GMP_PRO_LOCATION)/ctl/suite/mcs_pmsm_nt/src/ctl_main.c \
	$(GMP_PRO_LOCATION)/ctl/suite/mcs_pmsm_nt/src/user_main.c \
	xplt/xplt.peripheral.c \
	$(GMP_PRO_LOCATION)/csp/c29x_syscfg/src/sysconfig_adapter.c \
	$(GMP_PRO_LOCATION)/csp/c29x_syscfg/src/peripheral_driver.c \
	$(GMP_PRO_LOCATION)/core/std/src/gmp_std_port.c \
	$(GMP_PRO_LOCATION)/core/dev/src/gmp_datalink.c \
	$(GMP_PRO_LOCATION)/core/dev/src/gmp_dev_util.c \
	$(GMP_PRO_LOCATION)/core/dev/src/gmp_mem_presp.c \
	$(GMP_PRO_LOCATION)/core/dev/src/gmp_pil_core.c \
	$(GMP_PRO_LOCATION)/core/dev/src/gmp_scope.c \
	$(GMP_PRO_LOCATION)/core/dev/src/gmp_tunable.c \
	$(GMP_PRO_LOCATION)/core/pm/src/gmp_process_mgr.c \
	$(GMP_PRO_LOCATION)/ctl/component/dsa/src/dsa_dl_scope.c \
	$(GMP_PRO_LOCATION)/ctl/component/dsa/src/dsa_scope.c \
	$(GMP_PRO_LOCATION)/ctl/component/dsa/src/dsa_trigger.c \
	$(GMP_PRO_LOCATION)/ctl/component/interface/src/ctl_component_interface.c \
	$(GMP_PRO_LOCATION)/ctl/component/interface/src/ctl_if_pwm_spwm.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/basic/src/ctl_divider.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/basic/src/ctl_slope_limiter.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/complex/src/ctl_dq_pi.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/continuous/src/ctl_continuous_pid.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/discrete/src/ctl_discrete_biquad_filter.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/discrete/src/ctl_discrete_filter.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/discrete/src/ctl_discrete_lead_lag.c \
	$(GMP_PRO_LOCATION)/ctl/component/intrinsic/discrete/src/ctl_discrete_sg.c \
	$(GMP_PRO_LOCATION)/ctl/component/motor_control/basic/src/ctl_mc_mtr_protection.c \
	$(GMP_PRO_LOCATION)/ctl/component/motor_control/basic/src/ctl_mc_mtr_vf_generator.c \
	$(GMP_PRO_LOCATION)/ctl/component/motor_control/current_loop/src/ctl_mc_foc_core.c \
	$(GMP_PRO_LOCATION)/ctl/component/motor_control/interface/src/ctl_mc_encoder.c \
	$(GMP_PRO_LOCATION)/ctl/component/motor_control/mechanical_loop/src/ctl_mc_basic_mech_ctrl.c \
	$(GMP_PRO_LOCATION)/ctl/framework/src/ctl_cia402_callback_fn.c \
	$(GMP_PRO_LOCATION)/ctl/framework/src/ctl_cia402_state_machine.c \
	$(SYSCFG_OUT)/board.c \
	$(SYSCFG_OUT)/device.c \
	$(SYSCFG_OUT)/ti_drivers_config.c \
	$(SYSCFG_OUT)/ti_dpl_config.c \
	$(C29SDK_ROOT)/source/bitfields/bitfield_structs.c

OBJECTS := $(foreach source,$(SOURCES),$(OBJ)/$(notdir $(basename $(source))).o) $(OBJ)/codestartbranch.o

.PHONY: all syscfg clean
all: $(BUILD)/$(TARGET).out
syscfg: $(SYSCFG_OUT)/board.c

$(SYSCFG_OUT)/board.c: mcs_pmsm_nt_f29h85x_3phgan.syscfg
	@if not exist "$(SYSCFG_OUT)" mkdir "$(SYSCFG_OUT)"
	"$(SYSCFG)" --script "$<" -o "$(SYSCFG_OUT)" -s "$(C29SDK_ROOT)/.metadata/sdk.json" --compiler ticlang

$(SYSCFG_OUT)/device.c $(SYSCFG_OUT)/ti_drivers_config.c $(SYSCFG_OUT)/ti_dpl_config.c: $(SYSCFG_OUT)/board.c

$(OBJ):
	@if not exist "$(OBJ)" mkdir "$(OBJ)"

define make-c-rule
$(OBJ)/$(notdir $(basename $(1))).o: $(1) $(SYSCFG_OUT)/board.c | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$$@" "$$<"
endef
$(foreach source,$(SOURCES),$(eval $(call make-c-rule,$(source))))

$(OBJ)/codestartbranch.o: $(C29SDK_ROOT)/examples/device_support/source/codestartbranch.asm | $(OBJ)
	"$(COMPILER)" -c $(CFLAGS) -o "$@" "$<"

$(BUILD)/$(TARGET).out: $(OBJECTS)
	"$(COMPILER)" $(INCLUDES) $(LDFLAGS) $(LIB_PATHS) -Wl,-m="$(BUILD)/$(TARGET).map" -o "$@" $(OBJECTS) $(LIBS) "$(C29SDK_ROOT)/examples/device_support/cmd/f29h85x_flash_lnk.cmd"

clean:
	@echo Remove the build directory to clean this project.
