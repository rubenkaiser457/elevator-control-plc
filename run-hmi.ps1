#requires -Version 5
# Start the graphical HMI (Swing) and connect it to the MQTT broker.
# The HMI talks ONLY MQTT - it never imports control-system code.
#
#   .\run-hmi.ps1            # graphical HMI
#   .\run-hmi.ps1 -Console   # console HMI (headless/debug)
param([switch]$Console)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path

$creds = Join-Path $here "mqtt-credentials.ps1"
if (-not (Test-Path $creds)) {
    throw "Missing $creds - copy mqtt-credentials.example.ps1 to mqtt-credentials.ps1 and fill in the password."
}
. $creds

$main = if ($Console) { "de.htwg.sysarch.hmi.HmiApplication" } else { "de.htwg.sysarch.hmi.HmiSwingApplication" }

$mvn = if (Test-Path "C:\Tools\maven\bin\mvn.cmd") { "C:\Tools\maven\bin\mvn.cmd" }
       elseif (Get-Command mvn.cmd -ErrorAction SilentlyContinue) { "mvn.cmd" }
       else { "mvn" }
if (-not $env:JAVA_HOME -or -not (Test-Path $env:JAVA_HOME)) {
    $jdk = Get-ChildItem "C:\Program Files\Eclipse Adoptium" -Directory -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -like "jdk-11*" } | Select-Object -First 1
    if ($jdk) { $env:JAVA_HOME = $jdk.FullName }
}

Write-Host "Starting HMI ($main) -> $($env:MQTT_HOST):$($env:MQTT_PORT) base=$($env:MQTT_BASE_TOPIC)" -ForegroundColor Cyan
Push-Location (Join-Path $here "hmi")
try {
    & $mvn -q exec:java "-Dexec.mainClass=$main" `
        "-Dmqtt.host=$($env:MQTT_HOST)" "-Dmqtt.port=$($env:MQTT_PORT)" `
        "-Dmqtt.baseTopic=$($env:MQTT_BASE_TOPIC)" `
        "-Dmqtt.username=$($env:MQTT_USERNAME)" "-Dmqtt.password=$($env:MQTT_PASSWORD)"
} finally { Pop-Location }
