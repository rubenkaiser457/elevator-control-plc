# Copy this file to "mqtt-credentials.ps1" (which is gitignored) and fill in your
# group's broker credentials. The run-*.ps1 / deploy-control.ps1 scripts source it
# so secrets never live in git or on the command line.
#
#   cp mqtt-credentials.example.ps1 mqtt-credentials.ps1
#
$env:MQTT_HOST       = "localhost"                      # MQTT broker host
$env:MQTT_PORT       = "1883"
$env:MQTT_BASE_TOPIC = "/SysArch/E"                     # must match control and HMI
$env:MQTT_USERNAME   = ""                               # broker user (empty for anonymous)
$env:MQTT_PASSWORD   = ""                               # broker password
