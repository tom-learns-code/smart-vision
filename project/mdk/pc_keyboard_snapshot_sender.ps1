param(
    [string]$Port = "COM11",
    [int]$Baud = 115200
)

$ErrorActionPreference = "Stop"

Write-Host "PC keyboard snapshot sender"
Write-Host "Port: $Port"
Write-Host "Keys: s = save photo, p = ping, h = help, q = quit"
Write-Host ""
Write-Host "Important: disconnect OpenMV IDE first, otherwise COM port access may fail."
Write-Host ""

$serial = New-Object System.IO.Ports.SerialPort $Port, $Baud, "None", 8, "One"
$serial.ReadTimeout = 50
$serial.WriteTimeout = 1000
$serial.DtrEnable = $true
$serial.RtsEnable = $true

try {
    $serial.Open()
} catch {
    Write-Host "Failed to open $Port."
    Write-Host "Close/disconnect OpenMV IDE, then run this script again."
    Write-Host $_.Exception.Message
    exit 1
}

Write-Host "Opened $Port. Waiting for OpenART..."
Start-Sleep -Seconds 2

try {
    while ($true) {
        while ([Console]::KeyAvailable) {
            $key = [Console]::ReadKey($true)
            $ch = $key.KeyChar

            if ($ch -eq 'q' -or $ch -eq 'Q') {
                Write-Host "Quit."
                return
            } elseif ($ch -eq 's' -or $ch -eq 'S') {
                $serial.Write("s")
                Write-Host "[PC] sent: s"
            } elseif ($ch -eq 'p' -or $ch -eq 'P') {
                $serial.Write("p")
                Write-Host "[PC] sent: p"
            } elseif ($ch -eq 'h' -or $ch -eq 'H' -or $ch -eq '?') {
                $serial.Write("h")
                Write-Host "[PC] sent: h"
            }
        }

        try {
            $line = $serial.ReadLine()
            if ($line) {
                Write-Host ("[OpenART] " + $line.Trim())
            }
        } catch [System.TimeoutException] {
            # No incoming line right now.
        }

        Start-Sleep -Milliseconds 20
    }
} finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
