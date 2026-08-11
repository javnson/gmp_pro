$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $PSScriptRoot 'build'
New-Item -ItemType Directory -Force -Path $build | Out-Null

$common = @(
    (Join-Path $root 'src/iir2_operator.v'),
    (Join-Path $root 'src/iir2_pipeline_container.v'),
    (Join-Path $root 'src/iir2_axi_lite_regs.v'),
    (Join-Path $root 'src/iir2_control_core.v')
)

$tests = @(
    @{Name='tb_iir2_operator'; Sources=@($common[0])},
    @{Name='tb_iir2_pipeline'; Sources=@($common[0],$common[1])},
    @{Name='tb_control_core_axi'; Sources=$common},
    @{Name='tb_epwm_modulator'; Sources=@((Join-Path $root 'src/epwm_modulator.v'))},
    @{Name='tb_data_adapters'; Sources=@(
        (Join-Path $root 'src/adc_axis_adapter.v'),
        (Join-Path $root 'src/fixed_to_dac.v'))},
    @{Name='tb_spi_axi_bridge'; Sources=@($common + @(
        (Join-Path $root 'src/spi_axi_lite_bridge.v')))},
    @{Name='tb_external_memory_system'; Sources=@($common + @(
        (Join-Path $root 'src/axi4_stream_dma.v'),
        (Join-Path $root 'src/iir2_external_memory_system.v')))}
)

foreach ($test in $tests) {
    $output = Join-Path $build ($test.Name + '.vvp')
    $tb = Join-Path $PSScriptRoot ($test.Name + '.sv')
    & iverilog -g2005-sv -Wall -s $test.Name -o $output @($test.Sources) $tb
    if ($LASTEXITCODE -ne 0) { throw "iverilog failed: $($test.Name)" }
    & vvp $output
    if ($LASTEXITCODE -ne 0) { throw "simulation failed: $($test.Name)" }
}

$topOutput = Join-Path $build 'td_iir2_control_top.vvp'
$topSources = @(
    (Join-Path $PSScriptRoot 'clock_manager_stub.v'),
    (Join-Path $root 'src/ads8688_ctrl.v'),
    (Join-Path $root 'src/dac8563_qual_ctrl.v'),
    (Join-Path $root 'src/iir2_operator.v'),
    (Join-Path $root 'src/iir2_pipeline_container.v'),
    (Join-Path $root 'src/iir2_axi_lite_regs.v'),
    (Join-Path $root 'src/iir2_control_core.v'),
    (Join-Path $root 'src/adc_axis_adapter.v'),
    (Join-Path $root 'src/fixed_to_dac.v'),
    (Join-Path $root 'src/epwm_modulator.v'),
    (Join-Path $root 'src/spi_axi_lite_bridge.v'),
    (Join-Path $root 'src/axi4_stream_dma.v'),
    (Join-Path $root 'src/iir2_external_memory_system.v'),
    (Join-Path $root 'src/spi_top.v')
)
& iverilog -g2005-sv -Wall -s td_iir2_control_top -o $topOutput @topSources
if ($LASTEXITCODE -ne 0) { throw 'full TD top-level elaboration failed' }
Write-Host 'PASS td_iir2_control_top elaboration'

& python (Join-Path $PSScriptRoot 'test_host_tools.py')
if ($LASTEXITCODE -ne 0) { throw 'host-tool unit tests failed' }
