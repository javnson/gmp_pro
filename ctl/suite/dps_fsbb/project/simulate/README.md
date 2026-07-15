# FSBB MATLAB/Simulink SIL

This project runs the same FSBB control source used by the F280039C IRIS target against `MCS_STD_FSBB_MODEL.slx` over the GMP ASIO UDP protocol.

## SDPE and model mask

`sdpe_mgr/sdpe_requirement.json` is the source of truth for controller, power-stage, PWM, ADC, and sensor parameters. Generate both artifacts after changing it:

```powershell
python E:\lib\gmp_pro\tools\SDPE_v2\sdpe.py generate-project-local sdpe_mgr\sdpe_requirement.json --project-dir sdpe_mgr
python E:\lib\gmp_pro\tools\SDPE_v2\sdpe.py generate-project-matlab-local sdpe_mgr\sdpe_requirement.json --project-dir sdpe_mgr
```

The generated C header configures the Windows controller. The generated MATLAB script initializes the model and its `GMP STD FSBB Module` mask. Run `configure_fsbb_model` only when the mask bindings or model internals need to be recreated.

The sensor subsystems output raw ADC codes directly. The UDP ADC vector contract is:

1. input voltage (`Vin`)
2. output voltage (`Vout`)
3. inductor current (`IL`)
4. Boost-side output current (`Iout`)
5. Buck-side input current (`Iin`, diagnostic channel)

The controller consumes the first four channels. For a 12-bit bipolar current channel, zero current produces approximately 2048 because the configured sensor bias is 1.65 V.

## Build and run

Build `GMP_Motor_Control_simulink.sln` as `Debug|x64` with Visual Studio 2022. The required UDP S-function is loaded from `tools/gmp_sil/udp_helper_v2/mdl_asio_helper/bin/x64/Debug` by the model callback.

In MATLAB:

```matlab
cd('E:/lib/gmp_pro/ctl/suite/dps_fsbb/project/simulate');
out = run_fsbb_cosim(0.8);
```

The runner starts and stops the controller executable automatically. `network.json` defines the matching UDP ports: model-to-controller 12500, controller-to-model 12501, and command ports 12502/12503.

## Verified baseline

With the committed SDPE settings (24 V input, 24 V command, 20 ohm load, BUILD_LEVEL 3), a 0.8 s joint run completed 16,000 UDP control exchanges. The final quantized measurements were approximately Vin 23.98 V, Vout 23.38 V, and IL 1.21 A; Vout was still completing the configured 1 pu/s startup ramp and settling toward 24 V.
