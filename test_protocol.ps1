# PowerShell test script for PT-IO-Mini2 JSON protocol
# Usage: .\test_protocol.ps1 COM3

param(
    [Parameter(Mandatory=$true)]
    [string]$PortName
)

# Open serial port
$port = New-Object System.IO.Ports.SerialPort $PortName, 115200, None, 8, One
$port.Open()
Write-Host "Connected to $PortName" -ForegroundColor Green

Start-Sleep -Seconds 2

# Read initial hello message
Write-Host "`n=== Initial Messages ===" -ForegroundColor Cyan
while ($port.BytesToRead -gt 0) {
    $line = $port.ReadLine()
    Write-Host "← $line" -ForegroundColor Yellow
}

# Helper function to send JSON command
function Send-JsonCmd {
    param([string]$json)
    Write-Host "`n→ $json" -ForegroundColor Green
    $port.WriteLine($json)
    Start-Sleep -Milliseconds 200
    while ($port.BytesToRead -gt 0) {
        $response = $port.ReadLine()
        Write-Host "← $response" -ForegroundColor Yellow
    }
}

# Run tests
Write-Host "`n=== Test 1: getHello ===" -ForegroundColor Cyan
Send-JsonCmd '{"cmd":"getHello"}'

Write-Host "`n=== Test 2: getStatus ===" -ForegroundColor Cyan
Send-JsonCmd '{"cmd":"getStatus"}'

Write-Host "`n=== Test 3: setOutput (ch 1 to 50%) ===" -ForegroundColor Cyan
Send-JsonCmd '{"cmd":"setOutput","ch":1,"duty":50}'

Write-Host "`n=== Test 4: setSerialOverride (mask 0x07) ===" -ForegroundColor Cyan
Send-JsonCmd '{"cmd":"setSerialOverride","mask":7}'

Write-Host "`n=== Test 5: subscribe (1 Hz) ===" -ForegroundColor Cyan
Send-JsonCmd '{"cmd":"subscribe","stream":"telemetry","interval":1000}'

Write-Host "`n=== Receiving telemetry (5 seconds) ===" -ForegroundColor Cyan
$end = (Get-Date).AddSeconds(5)
while ((Get-Date) -lt $end) {
    if ($port.BytesToRead -gt 0) {
        $line = $port.ReadLine()
        Write-Host "← $line" -ForegroundColor Yellow
    }
    Start-Sleep -Milliseconds 100
}

Write-Host "`n=== Test 6: unsubscribe ===" -ForegroundColor Cyan
Send-JsonCmd '{"cmd":"unsubscribe"}'

Write-Host "`n=== Test 7: Text command (STATUS) ===" -ForegroundColor Cyan
Write-Host "→ STATUS" -ForegroundColor Green
$port.WriteLine("STATUS")
Start-Sleep -Milliseconds 200
while ($port.BytesToRead -gt 0) {
    $line = $port.ReadLine()
    Write-Host "← $line" -ForegroundColor Yellow
}

Write-Host "`n=== Tests Complete ===" -ForegroundColor Green
$port.Close()
