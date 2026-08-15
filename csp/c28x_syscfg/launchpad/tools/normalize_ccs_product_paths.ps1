param([string]$ProjectFile = (Join-Path (Split-Path -Parent $PSScriptRoot) '.cproject'))

$ErrorActionPreference = 'Stop'
$families = @{
    F2800137C = 'f280013x'
    F280025C  = 'f28002x'
    F280039C  = 'f28003x'
    F280049C  = 'f28004x'
    F28377S   = 'f2837xs'
    F28379D   = 'f2837xd'
    F28P55X   = 'f28p55x'
    F28P65X   = 'f28p65x'
}

$xml = [xml](Get-Content -Raw -LiteralPath $ProjectFile)
$xml.PreserveWhitespace = $true
$productImports = 'PRODUCT_MACRO_IMPORTS={"C2000WARE":["${COM_TI_C2000WARE_INCLUDE_PATH}","${COM_TI_C2000WARE_LIBRARY_PATH}","${COM_TI_C2000WARE_LIBRARIES}","${COM_TI_C2000WARE_SYMBOLS}","${COM_TI_C2000WARE_SYSCONFIG_MANIFEST}"],"GMP-Core-C28x":["${COM_TI_COM_GMP_CORE_C28X_SDK_INSTALL_DIR}","${COM_TI_COM_GMP_CORE_C28X_SDK_INCLUDE_PATH}","${COM_TI_COM_GMP_CORE_C28X_SDK_LIBRARY_PATH}","${COM_TI_COM_GMP_CORE_C28X_SDK_LIBRARIES}","${COM_TI_COM_GMP_CORE_C28X_SDK_SYMBOLS}","${COM_TI_COM_GMP_CORE_C28X_SDK_SYSCONFIG_MANIFEST}"]}'
$prebuildCommand = '"${PROJECT_LOC}/tools/ccs_prebuild.bat" "${ConfigName}" "${COM_TI_COM_GMP_CORE_C28X_SDK_INSTALL_DIR}" "${GMP_PREBUILD_SDPE}" "${GMP_PREBUILD_SRC_MGR}"'

# CCS stores tool options in both the core-settings configurations and a
# mirrored build-system tree.  Remove the override from both representations
# so a later IDE save cannot restore code_start as the ELF entry point.
@($xml.SelectNodes("//option[contains(@superClass,'.linkerID.ENTRY_POINT')]")) |
    ForEach-Object { [void]$_.ParentNode.RemoveChild($_) }
@($xml.SelectNodes('//sourceEntries/entry') | Where-Object {
    $_.GetAttribute('name') -match '/driverlib/(inc|ccs)$'
}) | ForEach-Object { [void]$_.ParentNode.RemoveChild($_) }

foreach ($configuration in $xml.SelectNodes('/cproject/storageModule[@moduleId="org.eclipse.cdt.core.settings"]/cconfiguration')) {
    $settingsNode = $configuration.SelectSingleNode("storageModule[@moduleId='org.eclipse.cdt.core.settings']")
    $name = $settingsNode.GetAttribute('name')
    $board = ($families.Keys | Where-Object { $name -like "${_}_*" } | Select-Object -First 1)
    if (-not $board) { throw "Unknown LaunchPad configuration: $name" }
    $family = $families[$board]

    $buildNode = $configuration.SelectSingleNode("storageModule[@moduleId='cdtBuildSystem']/configuration")
    $buildNode.SetAttribute('prebuildStep', $prebuildCommand)

    # These CCS build variables are independent and configuration-local.
    # Preserve an existing value so normalization does not reset a user's
    # Debug/Release selection.
    $macros = $settingsNode.SelectSingleNode('macros')
    if (-not $macros) {
        $macros = $xml.CreateElement('macros')
        [void]$settingsNode.PrependChild($macros)
    }
    foreach ($macroName in @('GMP_PREBUILD_SDPE', 'GMP_PREBUILD_SRC_MGR')) {
        $macro = $macros.SelectSingleNode("stringMacro[@name='$macroName']")
        if (-not $macro) {
            $macro = $xml.CreateElement('stringMacro')
            $macro.SetAttribute('name', $macroName)
            $macro.SetAttribute('type', 'VALUE_TEXT')
            $macro.SetAttribute('value', '0')
            [void]$macros.AppendChild($macro)
        }
    }
    # Let the C runtime keep its default ELF entry point (_c_int00).  The
    # Flash boot vector still enters the codestart section and code_start
    # branches to _c_int00, so forcing code_start here is both unnecessary
    # and the direct cause of TI linker warning #10063-D.
    # Compile the board-local DriverLib C sources directly.  Headers are
    # resolved from the registered C2000Ware product below; copied CCS
    # DriverLib projects and their prebuilt archives are deliberately excluded.
    $sourceEntries = $buildNode.SelectSingleNode('sourceEntries')
    $driverlibSource = "C2000Lib_$board/driverlib"
    $driverlibEntry = $sourceEntries.entry |
        Where-Object { $_.name -eq $driverlibSource } |
        Select-Object -First 1
    if (-not $driverlibEntry) {
        $driverlibEntry = $xml.CreateElement('entry')
        $driverlibEntry.SetAttribute('flags','VALUE_WORKSPACE_PATH|RESOLVED')
        $driverlibEntry.SetAttribute('kind','sourcePath')
        $driverlibEntry.SetAttribute('name',$driverlibSource)
        [void]$sourceEntries.AppendChild($driverlibEntry)
    }
    if ($driverlibEntry.HasAttribute('excluding')) {
        $driverlibEntry.RemoveAttribute('excluding')
    }

    $options = $buildNode.folderInfo.toolChain.option
    $tagOption = $options | Where-Object { $_.superClass -eq 'com.ti.ccstudio.buildDefinitions.core.OPT_TAGS' }
    foreach ($value in $tagOption.listOptionValue) {
        if ($value.value -like 'PRODUCTS=*') {
            $value.value = 'PRODUCTS=C2000WARE:5.4.0.00;GMP-Core-C28x:2.10.00.00;'
        } elseif ($value.value -like 'PRODUCT_MACRO_IMPORTS=*') {
            $value.value = $productImports
        }
    }

    $compiler = $buildNode.folderInfo.toolChain.tool |
        Where-Object { $_.superClass -like '*.compilerDebug' -or $_.superClass -like '*.compilerRelease' }
    $includeOption = $compiler.option | Where-Object { $_.superClass -like '*.compilerID.INCLUDE_PATH' }
    $kept = @($includeOption.listOptionValue | Where-Object {
        $_.value -notin @(
            '${PROJECT_LOC}/../../..',
            '${PROJECT_LOC}/..',
            '${GMP_PRO_ROOT}',
            '${GMP_C28X_CSP_ROOT}'
        ) -and
        $_.value -notlike '${PROJECT_LOC}/C2000Lib_*/driverlib' -and
        $_.value -notlike '${PROJECT_LOC}/C2000Lib_*/device_support/*'
    } | ForEach-Object { [string]$_.value })
    $portable = @(
        '${COM_TI_C2000WARE_INCLUDE_PATH}',
        '${COM_TI_COM_GMP_CORE_C28X_SDK_INCLUDE_PATH}',
        "`${COM_TI_C2000WARE_INSTALL_DIR}/device_support/$family/headers/include",
        "`${COM_TI_C2000WARE_INSTALL_DIR}/device_support/$family/common/include",
        "`${COM_TI_C2000WARE_INSTALL_DIR}/driverlib/$family/driverlib"
    )
    @($includeOption.ChildNodes) | ForEach-Object { [void]$includeOption.RemoveChild($_) }
    foreach ($path in @($portable + $kept | Select-Object -Unique)) {
        $node = $xml.CreateElement('listOptionValue')
        $node.SetAttribute('builtIn','false')
        $node.SetAttribute('value',$path)
        [void]$includeOption.AppendChild($node)
    }

    $defineOption = $compiler.option | Where-Object { $_.superClass -like '*.compilerID.DEFINE' }
    @($defineOption.listOptionValue | Where-Object value -eq 'GMP_CSP_C28X=1') |
        ForEach-Object { [void]$defineOption.RemoveChild($_) }
    if (-not ($defineOption.listOptionValue | Where-Object value -eq '${COM_TI_COM_GMP_CORE_C28X_SDK_SYMBOLS}')) {
        $node = $xml.CreateElement('listOptionValue')
        $node.SetAttribute('builtIn','false')
        $node.SetAttribute('value','${COM_TI_COM_GMP_CORE_C28X_SDK_SYMBOLS}')
        [void]$defineOption.AppendChild($node)
    }

    $sysconfig = $buildNode.folderInfo.toolChain.tool |
        Where-Object { $_.superClass -eq 'com.ti.ccstudio.buildDefinitions.sysConfig' }
    $sysProducts = $sysconfig.option | Where-Object { $_.superClass -eq 'com.ti.ccstudio.buildDefinitions.sysConfig.PRODUCTS' }
    if (-not ($sysProducts.listOptionValue | Where-Object value -eq '${COM_TI_COM_GMP_CORE_C28X_SDK_SYSCONFIG_MANIFEST}')) {
        $node = $xml.CreateElement('listOptionValue')
        $node.SetAttribute('builtIn','false')
        $node.SetAttribute('value','${COM_TI_COM_GMP_CORE_C28X_SDK_SYSCONFIG_MANIFEST}')
        [void]$sysProducts.AppendChild($node)
    }
}

$settings = [System.Xml.XmlWriterSettings]::new()
$settings.Indent = $true
$settings.IndentChars = "`t"
$settings.NewLineChars = "`r`n"
$settings.Encoding = [System.Text.UTF8Encoding]::new($false)
$writer = [System.Xml.XmlWriter]::Create((Resolve-Path $ProjectFile), $settings)
$xml.Save($writer)
$writer.Dispose()
