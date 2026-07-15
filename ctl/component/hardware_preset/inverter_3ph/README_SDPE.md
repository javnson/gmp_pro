# Three-phase inverter SDPE migration

The headers in this directory are the legacy control presets.  New projects shall select the corresponding
`inverter_3ph` entity from `ctl/hardware_preset/sdpe_src/inverter_3ph`; generated, normalized headers are written
to `ctl/hardware_preset/inverter_3ph`.

The canonical schema deliberately separates four different hardware classes:

- discrete MOSFET or IGBT (`power_switch`);
- packaged half-bridge power blocks (`power_switch` with a non-single configuration);
- integrated motor power stages (`integrated_power_stage`);
- intelligent three-phase power modules (`integrated_power_module`).

A shunt resistor and its amplifier are board signal-chain data, not a `current_sensor` entity.  A current-sensor
entity is attached only when the board contains a current-sensor IC, such as TMCS1133.

## Legacy-to-SDPE map

| Legacy preset | SDPE entity | Power stage classification | Current acquisition |
| --- | --- | --- | --- |
| `GMP_3PH_2136SINV_DUAL*.h` | `gmp_3ph_2136sinv_dual` | discrete MOSFET | 3 low-side shunts |
| `GMP_Helios_3PhGaNInv_LV.h` | `gmp_helios_3phganinv_lv` | discrete GaN | 3 inline Hall sensors plus DC-link shunt |
| `ST_B_G431B_ESC1.h` | `st_b_g431b_esc1` | discrete MOSFET | 3 low-side shunts |
| `ST_EVLSERVO1.H` | `st_evlservo1` | discrete MOSFET | 3 low-side shunts |
| `ST_NUCLEO_IHM08M1.h` | `st_x_nucleo_ihm08m1` | discrete MOSFET | selectable 3-shunt / single-shunt |
| `ST_NUCLEO_IMH07M1.h` | `st_x_nucleo_ihm07m1` | integrated L6230 stage | selectable 3-shunt / single-shunt |
| `ST_NUCLEO-IHM11M1.h` | `st_x_nucleo_ihm11m1` | integrated STSPIN230 stage | single DC-link shunt |
| `ST_STEVAL-IHM023V3.h` | `st_steval_ihm023v3` | discrete IGBT | single DC-link shunt |
| `ST_STEVAL-IHM032V1.h` | `st_steval_ihm032v1` | discrete IGBT | single DC-link shunt |
| `ST_STEVAL-IPM20B.h` | `st_steval_ipm20b` | STGIB20M60TS-L IPM | selectable 3-shunt / single-shunt |
| `ST_STEVAL-IPMNG8Q.h` | `st_steval_ipmng8q` | STGIPQ8C60T-HZ IPM | selectable 3-shunt / single-shunt |
| `TI_BOOSTXL_3PhGaNInv.h` | `ti_boostxl_3phganinv` | three LMG5200 stages | 3 inline shunts with INA240 |
| `TI_BOOSTXL_DRV8301.h` | `ti_boostxl_drv8301` | discrete MOSFET | 3 low-side shunts |
| `TI_BOOSTXL_DRV8304H.h` | `ti_boostxl_drv8304h` | CSD88584Q5DC power blocks | 3 low-side shunts |
| `TI_BOOSTXL_DRV8305.h` | `ti_boostxl_drv8305` | discrete MOSFET | 3 low-side shunts |
| `TI_BOOSTXL_DRV8320RS.h` | `ti_boostxl_drv8320rs` | CSD88584Q5DC power blocks | 3 low-side shunts |
| `TI_BOOSTXL_DRV8323RS.h` | `ti_boostxl_drv8323rs` | CSD88584Q5DC power blocks | 3 low-side shunts |
| `ZR_DRV8301.h` | `zr_drv8301` | switch identity unknown | placement requires schematic |

## Data-quality rules

`*_VERIFICATION_STATUS` and `*_PWM_POLARITY_VERIFIED` must be checked before a preset is used to configure
protection or gate logic.  A zero electrical limit means “not specified”, not a zero-rated device.  In particular:

- the exact Helios TMCS1133 BOM suffix and INN100E0016A device data still require the released BOM/datasheet;
- the old text `TMCS1133A4B` is not a valid published TI orderable name; the registered 100 mV/A, 1.65 V option is
  `tmcs1133_b4a`;
- ZR_DRV8301 switch identity, current-shunt placement and PWM polarity remain unverified;
- values marked `partial` must not be promoted to protection thresholds without the board schematic.

Primary references are stored in every entity's `datasheet_url` or `source_reference`.  Representative board
documents include ST UM1996/UM1943/UM2095/UM1823/UM1078/UM2280 and TI SLUUBP1A/SLVU974/SLVUB97/SLVUAI8A.
