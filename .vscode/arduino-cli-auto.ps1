param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("upload", "monitor", "detect")]
    [string]$Mode,

    [string]$SketchPath,
    [string]$Fqbn = "esp32:esp32:esp32thing",
    [string]$Baudrate = "115200",
    [string]$UploadSpeed = "921600"
)

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
        (($_.port.properties.vid -eq "0x0403") -and ($_.port.properties.pid -eq "0x6010")) -or
        (($_.port.properties.vid -eq "0x0403") -and ($_.port.properties.pid -eq "0x6014"))
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

$port = $selected.port.address
Write-Host "Using serial port: $port (VID=$($selected.port.properties.vid) PID=$($selected.port.properties.pid))"

if ($Mode -eq "detect") {
    exit 0
}

if ($Mode -eq "upload") {
    if (-not $SketchPath) {
        throw "SketchPath is required for upload mode"
    }

    arduino-cli upload -p $port --fqbn $Fqbn --upload-property "upload.speed=$UploadSpeed" $SketchPath
    exit $LASTEXITCODE
}

arduino-cli monitor -p $port --config "baudrate=$Baudrate"
exit $LASTEXITCODE
