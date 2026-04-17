param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("upload", "upload-usb", "monitor", "monitor-usb", "detect")]
    [string]$Mode,

    [string]$SketchPath,
    [string]$Fqbn = "esp32:esp32:esp32thing",
    [string]$Baudrate = "115200",
    [string]$UploadSpeed = "921600"
)

$sparkfunVid = "0x0403"
$sparkfunPid = "0x6015"
$espProgVid = "0x0403"
$espProgPidA = "0x6010"
$espProgPidB = "0x6014"

function Normalize-HexId([string]$value) {
    if (-not $value) { return "" }
    $v = $value.Trim().ToLowerInvariant()
    if ($v.StartsWith("0x")) {
        $v = $v.Substring(2)
    }
    $v = $v.TrimStart('0')
    if (-not $v) { $v = "0" }
    return "0x$v"
}

function Test-VidPid([object]$portObj, [string]$targetVid, [string]$targetPid) {
    $portVid = Normalize-HexId $portObj.port.properties.vid
    $portPid = Normalize-HexId $portObj.port.properties.pid
    return ($portVid -eq (Normalize-HexId $targetVid)) -and ($portPid -eq (Normalize-HexId $targetPid))
}

function Get-OrderedPortCandidates([string]$mode, [array]$allPorts) {
    $ordered = New-Object System.Collections.ArrayList

    function Add-Candidates([array]$matches) {
        foreach ($m in @($matches)) {
            if (-not $m) { continue }
            $addr = $m.port.address
            if (-not $addr) { continue }
            if (-not ($ordered | Where-Object { $_.port.address -eq $addr })) {
                [void]$ordered.Add($m)
            }
        }
    }

    if (($mode -eq "upload-usb") -or ($mode -eq "monitor-usb")) {
        # Prefer SparkFun ESP32 Thing USB CDC UART first.
        Add-Candidates ($allPorts | Where-Object { Test-VidPid $_ $sparkfunVid $sparkfunPid })

        # Then prefer non-ESP-Prog USB serial adapters for direct USB uploads.
        Add-Candidates ($allPorts | Where-Object {
            ($_.port.properties.vid -or $_.port.properties.pid) -and -not (
                (Test-VidPid $_ $espProgVid $espProgPidA) -or
                (Test-VidPid $_ $espProgVid $espProgPidB)
            )
        })

        Add-Candidates ($allPorts | Where-Object { $_.port.protocol_label -match "USB" })
        Add-Candidates ($allPorts | Where-Object { $_.port.address -ne "COM1" })
        Add-Candidates $allPorts
    }
    else {
        # Prefer ESP-Prog (FTDI FT2232H) first.
        Add-Candidates ($allPorts | Where-Object {
            (Test-VidPid $_ $espProgVid $espProgPidA) -or
            (Test-VidPid $_ $espProgVid $espProgPidB)
        })

        Add-Candidates ($allPorts | Where-Object { $_.port.properties.vid -or $_.port.properties.pid })
        Add-Candidates ($allPorts | Where-Object { $_.port.protocol_label -match "USB" })
        Add-Candidates ($allPorts | Where-Object { $_.port.address -ne "COM1" })
        Add-Candidates $allPorts
    }

    return @($ordered)
}

function Stop-ArduinoMonitorProcesses {
    $monitorProcs = Get-CimInstance Win32_Process |
        Where-Object {
            ($_.Name -match "^arduino-cli(\\.exe)?$") -and
            ($_.CommandLine -match "\\smonitor(\\s|$)")
        }

    foreach ($p in @($monitorProcs)) {
        if (-not $p.ProcessId) { continue }
        Write-Host "Stopping existing monitor process PID=$($p.ProcessId) to free serial port..."
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
}

$boardListJson = arduino-cli board list --format json
if (-not $boardListJson) {
    throw "Unable to read ports from arduino-cli board list"
}

$boardList = $boardListJson | ConvertFrom-Json
$ports = @($boardList.detected_ports | Where-Object { $_.port.address })
if ($ports.Count -eq 0) {
    throw "No serial ports detected"
}

$orderedCandidates = Get-OrderedPortCandidates -mode $Mode -allPorts $ports
$selected = $orderedCandidates | Select-Object -First 1
if (-not $selected) {
    throw "No candidate serial ports available"
}

$port = $selected.port.address
Write-Host "Using serial port: $port (VID=$($selected.port.properties.vid) PID=$($selected.port.properties.pid))"

if ($Mode -eq "detect") {
    exit 0
}

if (($Mode -eq "upload") -or ($Mode -eq "upload-usb")) {
    if (-not $SketchPath) {
        throw "SketchPath is required for upload mode"
    }

    # Ensure no stale arduino-cli monitor instance keeps the serial port open.
    Stop-ArduinoMonitorProcesses

    $uploadTargets = @($selected)
    if ($Mode -eq "upload-usb") {
        $uploadTargets = @($orderedCandidates)
    }

    $attempt = 0
    foreach ($candidate in $uploadTargets) {
        $attempt++
        $candidatePort = $candidate.port.address
        Write-Host "Upload attempt $attempt on $candidatePort..."

        arduino-cli upload -p $candidatePort --fqbn $Fqbn --upload-property "upload.speed=$UploadSpeed" $SketchPath
        if ($LASTEXITCODE -eq 0) {
            exit 0
        }

        if ($Mode -ne "upload-usb") {
            exit $LASTEXITCODE
        }

        Write-Host "Upload failed on $candidatePort (exit=$LASTEXITCODE). Trying next candidate, if available..."
    }

    throw "Upload failed on all candidate ports for mode '$Mode'."
}

arduino-cli monitor -p $port --config "baudrate=$Baudrate"
exit $LASTEXITCODE
