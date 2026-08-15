param([string]$ProjectFile = (Join-Path (Split-Path -Parent $PSScriptRoot) '.cproject'))

$ErrorActionPreference = 'Stop'
$matrix = @(
    @{ Board='F2800137C'; Device='TMS320F2800137';  Package='64PM';   Family='f280013x'; Symbol='_LAUNCHXL_F2800137' },
    @{ Board='F280025C';  Device='TMS320F280025C';  Package='80QFP';  Family='f28002x';  Symbol='_LAUNCHXL_F280025C' },
    @{ Board='F280039C';  Device='TMS320F280039C';  Package='100PZ';  Family='f28003x';  Symbol='_LAUNCHXL_F280039C' },
    @{ Board='F280049C';  Device='TMS320F280049C';  Package='100PZ';  Family='f28004x';  Symbol='_LAUNCHXL_F280049C' },
    @{ Board='F28377S';   Device='TMS320F28377S';   Package='176PTP'; Family='f2837xs';  Symbol='_LAUNCHXL_F28377S'; ExtraDefines=@('SYSCTL_EMIF1CLK_DIV_SYSCTL_EMIF1CLK_DIV_1=SYSCTL_EMIF1CLK_DIV_1') },
    @{ Board='F28379D';   Device='TMS320F28379D';   Package='176PTP'; Family='f2837xd';  Symbol='_LAUNCHXL_F28379D'; ExtraDefines=@('SYSCTL_EMIF1CLK_DIV_SYSCTL_EMIF1CLK_DIV_1=SYSCTL_EMIF1CLK_DIV_1') },
    @{ Board='F28P55X';   Device='TMS320F28P550SJ'; CcsDevice='TMS320F28P550SJ9'; Package='100PZ';  Family='f28p55x';  Symbol='_LAUNCHXL_F28P55X' },
    @{ Board='F28P65X';   Device='TMS320F28P650DK'; CcsDevice='TMS320F28P650DK9'; Package='169NMR'; Family='f28p65x';  Symbol='_LAUNCHXL_F28P65X' }
)

$xml = [xml](Get-Content -Raw -LiteralPath $ProjectFile)
$xml.PreserveWhitespace = $true
$rootSettings = $xml.SelectSingleNode('/cproject/storageModule[@moduleId="org.eclipse.cdt.core.settings"]')
$debugTemplate = $rootSettings.SelectSingleNode("cconfiguration[storageModule[@name='F280049C_Debug']]")
$releaseTemplate = $rootSettings.SelectSingleNode("cconfiguration[storageModule[@name='F280049C_Release']]")
if (-not $debugTemplate -or -not $releaseTemplate) { throw 'F280049C template configurations not found' }

@($rootSettings.SelectNodes('cconfiguration')) | ForEach-Object { [void]$rootSettings.RemoveChild($_) }

foreach ($item in $matrix) {
    foreach ($mode in @('Debug','Release')) {
        $template = if ($mode -eq 'Debug') { $debugTemplate } else { $releaseTemplate }
        $node = $template.CloneNode($true)
        $suffix = ($item.Board + $mode).ToLowerInvariant()
        $idMap = @{}
        foreach ($element in @($node.SelectNodes('.//*[@id]')) + @($node)) {
            if ($element.HasAttribute('id')) {
                $old = $element.GetAttribute('id')
                $new = "$old.$suffix"
                $idMap[$old] = $new
                $element.SetAttribute('id',$new)
            }
        }
        foreach ($element in $node.SelectNodes('.//*')) {
            foreach ($attribute in @($element.Attributes)) {
                if ($idMap.ContainsKey($attribute.Value)) { $attribute.Value = $idMap[$attribute.Value] }
                $attribute.Value = $attribute.Value.Replace('F280049C',$item.Board).Replace('f28004x',$item.Family).Replace('_LAUNCHXL_F280049C',$item.Symbol)
            }
        }
        $settingsNode = $node.SelectSingleNode("storageModule[@moduleId='org.eclipse.cdt.core.settings']")
        $settingsNode.SetAttribute('name',"$($item.Board)_$mode")
        $build = $node.SelectSingleNode("storageModule[@moduleId='cdtBuildSystem']/configuration")
        $build.SetAttribute('name',"$($item.Board)_$mode")
        foreach ($value in $build.SelectNodes("folderInfo/toolChain/option[@superClass='com.ti.ccstudio.buildDefinitions.core.OPT_TAGS']/listOptionValue")) {
            if ($value.GetAttribute('value') -like 'DEVICE_CONFIGURATION_ID=*') {
                $ccsDevice = if ($item.CcsDevice) { $item.CcsDevice } else { $item.Device }
                $value.SetAttribute('value',"DEVICE_CONFIGURATION_ID=TMS320C28XX.$ccsDevice")
            }
        }
        $sysconfig = $build.SelectSingleNode("folderInfo/toolChain/tool[@superClass='com.ti.ccstudio.buildDefinitions.sysConfig']")
        $sysconfig.SelectSingleNode("option[@superClass='com.ti.ccstudio.buildDefinitions.sysConfig.DEVICE']").SetAttribute('value',$item.Device)
        $sysconfig.SelectSingleNode("option[@superClass='com.ti.ccstudio.buildDefinitions.sysConfig.PACKAGE']").SetAttribute('value',$item.Package)
        $compilerDefines = $build.SelectSingleNode("folderInfo/toolChain/tool[starts-with(@superClass,'com.ti.ccstudio.buildDefinitions.C2000_') and contains(@superClass,'.compiler')]/option[contains(@superClass,'.compilerID.DEFINE')]")
        foreach ($define in @($item.ExtraDefines)) {
            $defineNode = $xml.CreateElement('listOptionValue')
            $defineNode.SetAttribute('builtIn','false')
            $defineNode.SetAttribute('value',$define)
            [void]$compilerDefines.AppendChild($defineNode)
        }
        foreach ($entry in $build.SelectNodes('sourceEntries/entry')) {
            if ($entry.GetAttribute('name') -like 'C2000Lib_*') {
                $entry.SetAttribute('name',$entry.GetAttribute('name').Replace('C2000Lib_F280049C',"C2000Lib_$($item.Board)"))
                if ($entry.HasAttribute('excluding')) { $entry.SetAttribute('excluding','build_support|device_cmd|device_support|driverlib|hardware_preset|hw|sdpe|targetConfigs') }
            }
        }
        [void]$rootSettings.AppendChild($node)
    }
}

$writerSettings = [System.Xml.XmlWriterSettings]::new()
$writerSettings.Indent = $true
$writerSettings.IndentChars = "`t"
$writerSettings.NewLineChars = "`r`n"
$writerSettings.Encoding = [System.Text.UTF8Encoding]::new($false)
$writer = [System.Xml.XmlWriter]::Create((Resolve-Path $ProjectFile),$writerSettings)
$xml.Save($writer)
$writer.Dispose()
& (Join-Path $PSScriptRoot 'normalize_ccs_product_paths.ps1') -ProjectFile $ProjectFile
