[CmdletBinding()]
param(
    [string]$Programmer = "",
    [string]$ExternalLoader = ""
)

$ErrorActionPreference = "Stop"
if (-not $Programmer) {
    $Programmer = Join-Path $env:ProgramFiles "STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
}
if (-not $ExternalLoader) {
    $ExternalLoader = Join-Path $env:ProgramFiles "STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\ExternalLoader\MX25UM51245G_STM32N6570-NUCLEO.stldr"
}

$firmwareDirectory = Resolve-Path (Join-Path $PSScriptRoot "..\firmware")
$fsbl = Join-Path $firmwareDirectory "ai_fsbl.hex"
$application = Join-Path $firmwareDirectory "NUCLEO-N657X0-Q_GettingStarted_ObjectDetection_signed.bin"
$network = Join-Path $firmwareDirectory "network_atonbuf.xSPI2.bin"

foreach ($requiredPath in @($Programmer, $ExternalLoader, $fsbl, $application, $network)) {
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "Required file not found: $requiredPath"
    }
}

function Invoke-Programmer {
    param([string[]]$ProgrammerArguments)

    & $Programmer @ProgrammerArguments
    if ($LASTEXITCODE -ne 0) {
        throw "STM32CubeProgrammer failed with exit code $LASTEXITCODE"
    }
}

Write-Host "Board must be connected in DEV BOOT mode."
Invoke-Programmer -ProgrammerArguments @("-c", "port=SWD", "mode=HOTPLUG", "-el", $ExternalLoader, "-hardRst", "-w", $fsbl)
Invoke-Programmer -ProgrammerArguments @("-c", "port=SWD", "mode=HOTPLUG", "-el", $ExternalLoader, "-hardRst", "-w", $application, "0x70100000")
Invoke-Programmer -ProgrammerArguments @("-c", "port=SWD", "mode=HOTPLUG", "-el", $ExternalLoader, "-hardRst", "-w", $network, "0x70380000")
Write-Host "Programming complete. Switch to FLASH BOOT and power-cycle the board."
