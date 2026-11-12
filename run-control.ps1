#requires -Version 5
# Start the control system locally with the in-memory PLC simulation and expose it to the
# HMI over MQTT.
#
#   .\run-control.ps1           # --sim --mqtt
#   .\run-control.ps1 -Modbus   # --modbus --mqtt (needs a reachable PLC and the easymodbus library)
param([switch]$Modbus)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

$creds = Join-Path $here "mqtt-credentials.ps1"
if (-not (Test-Path $creds)) {
    throw "Missing $creds - copy mqtt-credentials.example.ps1 to mqtt-credentials.ps1 and fill in the password."
}
. $creds

$plc = if ($Modbus) { "--modbus" } else { "--sim" }

$mvn = if (Test-Path "C:\Tools\maven\bin\mvn.cmd") { "C:\Tools\maven\bin\mvn.cmd" }
       elseif (Get-Command mvn.cmd -ErrorAction SilentlyContinue) { "mvn.cmd" }
       else { "mvn" }
if (-not $env:JAVA_HOME -or -not (Test-Path $env:JAVA_HOME)) {
    $jdk = Get-ChildItem "C:\Program Files\Eclipse Adoptium" -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -like "jdk-11*" } | Select-Object -First 1
    if ($jdk) { $env:JAVA_HOME = $jdk.FullName }
}

Write-Host "Starting control ($plc --mqtt) -> $($env:MQTT_HOST):$($env:MQTT_PORT) base=$($env:MQTT_BASE_TOPIC)" -ForegroundColor Cyan
Push-Location (Join-Path $here "control")
try {
    & $mvn -q exec:java "-Dexec.args=$plc --mqtt" `
        "-Dmqtt.host=$($env:MQTT_HOST)" "-Dmqtt.port=$($env:MQTT_PORT)" `
        "-Dmqtt.baseTopic=$($env:MQTT_BASE_TOPIC)" `
        "-Dmqtt.username=$($env:MQTT_USERNAME)" "-Dmqtt.password=$($env:MQTT_PASSWORD)"
} finally { Pop-Location }
