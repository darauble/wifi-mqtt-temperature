# wifi-mqtt-temperature
A simple temperature reading via DS18B20 and publishing on MQTT
designed for NodeMCU.

## First time use and reset
On startup WiFi hotspot is created to configure access point and
MQTT server credentials.

To reset your node press "flash" button or short D3 to ground for 5+
seconds. Afterwards device will reboot and start a hotspot again
for reconfiguration.

## Flashing
Flash via USB, but Arduino OTA is included for convenience too.

## Workflow
Software reads DS18B20 temperature sensors on several pins, currently
D1, D2 and D4. Several pins are useful when sensor network is arranged
in a star pattern to minimze overall (summed up) wire length for one
bus. Failures are also distributed in this case.

On startup each bus is scanned for sensors. Maximum number of overall
sensors is defined as 20, but this count can be increased if required.

Sensors are read every 30 seconds. Every 5 minutes all buses are
scanned again, so hot-wiring new sensors and removing failed once
is possible without disconecting the node.

# MQTT Topics
Heartbeat (lwt) and sensors are published. For each sensors full
scratchpad and human readable float temperature is published like
this, using ESP ID for heartbeat and sensor addresses:

```darauble/wifitemp/AC5359/hb online
darauble/ds18x20/28FF4011811604F8/scratchpad 6A014B467FFF0C10B8
darauble/ds18x20/28FF4011811604F8/temperature 22.62500
darauble/ds18x20/28FF36407516044D/scratchpad 69014B467FFF0C107D
darauble/ds18x20/28FF36407516044D/temperature 22.56250
darauble/ds18x20/28FFAC2880160473/scratchpad 69014B467FFF0C107D
darauble/ds18x20/28FFAC2880160473/temperature 22.56250
darauble/ds18x20/28FF1A1C801604E6/scratchpad 6C014B467FFF0C102B
darauble/ds18x20/28FF1A1C801604E6/temperature 22.75000
darauble/ds18x20/28FF891C8016041F/scratchpad 6D014B467FFF0C1068
darauble/ds18x20/28FF891C8016041F/temperature 22.81250
darauble/ds18x20/28FF4011811604F8/scratchpad 6B014B467FFF0C10FB
darauble/ds18x20/28FF4011811604F8/temperature 22.68750
darauble/ds18x20/28FF36407516044D/scratchpad 6A014B467FFF0C10B8
darauble/ds18x20/28FF36407516044D/temperature 22.62500```


