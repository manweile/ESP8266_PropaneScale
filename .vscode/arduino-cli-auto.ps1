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

$boardListJson = arduino-cli board list --format json
if (-not $boardListJson) {
    throw "Unable to read ports from arduino-cli board list"
}

$boardList = $boardListJson | ConvertFrom-Json
$ports = @($boardList.detected_ports | Where-Object { $_.port.address })
if ($ports.Count -eq 0) {
    throw "No serial ports detected"
}

# Prefer ESP-Prog (FTDI FT2232H) VID/PID first, then generic USB serial with VID/PID,
# then USB-labeled ports, then first available non-COM1 port, then first available.
$selected = $ports |
    Where-Object {
        (Test-VidPid $_ $espProgVid $espProgPidA) -or
        (Test-VidPid $_ $espProgVid $espProgPidB)
    } |
    Select-Object -First 1

if (-not $selected) {
    $selected = $ports | Where-Object { $_.port.properties.vid -or $_.port.properties.pid } | Select-Object -First 1
}
if (-not $selected) {
    $selected = $ports | Where-Object { $_.port.protocol_label -match "USB" } | Select-Object -First 1
}
if (-not $selected) {
    $selected = $ports | Where-Object { $_.port.address -ne "COM1" } | Select-Object -First 1
}
if (-not $selected) {
    $selected = $ports | Select-Object -First 1
}

if (($Mode -eq "upload-usb") -or ($Mode -eq "monitor-usb")) {
    # Prefer SparkFun ESP32 Thing USB CDC UART first.
    $selected = $ports |
        Where-Object { Test-VidPid $_ $sparkfunVid $sparkfunPid } |
        Select-Object -First 1

    if (-not $selected) {
        # Then prefer non-ESP-Prog USB serial adapters for direct USB uploads.
        $selected = $ports |
            Where-Object {
                ($_.port.properties.vid -or $_.port.properties.pid) -and -not (
                    (Test-VidPid $_ $espProgVid $espProgPidA) -or
                    (Test-VidPid $_ $espProgVid $espProgPidB)
                )
            } |
            Select-Object -First 1
    }

    if (-not $selected) {
        $selected = $ports | Where-Object { $_.port.protocol_label -match "USB" } | Select-Object -First 1
    }
    if (-not $selected) {
        $selected = $ports | Where-Object { $_.port.address -ne "COM1" } | Select-Object -First 1
    }
    if (-not $selected) {
        $selected = $ports | Select-Object -First 1
    }
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

    arduino-cli upload -p $port --fqbn $Fqbn --upload-property "upload.speed=$UploadSpeed" $SketchPath
    exit $LASTEXITCODE
}

arduino-cli monitor -p $port --config "baudrate=$Baudrate"
exit $LASTEXITCODE
