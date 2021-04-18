# ESP32 Garage Manager

## Overview

![front1](/img/ESP32-garage-front1.jpg)

![front2](/img/ESP32-garage-front2.jpg)

![back](/img/ESP32-garage-back.jpg)

## Basic features

- Remotely trigger a relay to open the garage door
- Magentic reed switch is used to determine whether the garage door is open or closed and send events when the state changes
- AHT10 temperature and humidity sensor sends regular updates (BME280 can be used instead using functions/temp-bme280.h instead of temp-ah10.h)
- Water leak sensor via a capacitive touch pin, intended to monitor a hot water heater for leaks
- OLED display alternates between Temp/Humidity and basic information (IP, device name, WiFi connection state, WiFi signal strength, door state, leak state)
- MQTT is used to publish and subscribe messages
- Node-RED interface to MQTT to process data, export Prometheus metrics, and send alerts for leaks or door left open at night
- Home Assistant and phone app is used for dashboard button to trigger door


## Parts

- [ESP32 devkit - 30 pin, integrated antenna](https://www.aliexpress.com/item/1005001267643044.html)
- [0.96" OLED - white, I2C](https://www.aliexpress.com/item/32896971385.html)
- [BME280 sensor](https://www.aliexpress.com/item/4001098967210.html) or [AHT10 sensor](https://www.aliexpress.com/item/4000125110813.html)
- [relay](https://www.aliexpress.com/item/32908761529.html)
- [magnetic reed switch](https://www.aliexpress.com/item/4001283716814.html)
- [passive piezo buzzer](https://www.aliexpress.com/item/4001317667682.html) or anything to make noise
- [alarm wire](https://www.amazon.com/gp/product/B01CT06M0A/) (500') - Used for magnetic reed switch connection, but useful for other connections.  It's 22 awg solid wire, which is easier to work with and solder than some of the cheap stranded wire.

## Diagram

![diagram](/img/diagram.png)

## Node-RED

![Node-RED flow](/node-red/node-red-flow.png)
[JSON for above flow](node-red/flow.json)

The beginnig of the flow starts out very simple.  Node-RED listens on the MQTT topic for the sensor.  The messages get converted from JSON to payload objects.

To understand the rest of the flow, it's important to go over how the Prometheus node exporter works in Node-RED.  The orange metric boxes to the right-side of the flow generate the prometheus data shown below at \<node_red_url\>:1880/metrics.

```
# HELP garage_temp garage temp
# TYPE garage_temp gauge
garage1_temp 83.1

# HELP garage_humidity garage humidity
# TYPE garage_humidity gauge
garage_humidity 22.58

# HELP garage_signal garage signal
# TYPE garage_signal gauge
garage_signal -61

# HELP garage_door garage door
# TYPE garage_door gauge
garage_door 0

# HELP garage_leak garage leak
# TYPE garage_leak gauge
garage_leak 0
```

The challenge has been dealing with sensors that go offline.  The metric node would not receive an updated value and continue to display an old value.  This would show up in the graphs in Grafana as a flat-line.  There might be a more elegant solution, but what I've done for now is to use an inject node to regularly inject a healthcheck message into the flow.  The healthcheck function node uses node context to store the time of the last valid message that came from MQTT.  When the healthcheck message is injected, the time of the inject message is compared to the time of the last valid message.  If the time difference is greater than the threshold, the healthcheck node's status is set to "offline" and a new message is generated and passed along with the metric values set to zero.  The Grafana graphs are set to only display values greater than zero.

The functions to the left of each of the orange metric nodes simply reformat the payload messages to set the metric values as required for the Promtheus metric nodes.

The "Check if open" inject node is scheduled to check if the garage door is open at night.  There are 2 alert processing functions that send messages to Slack (door open and leak events).

Below is an example graph in Grafana.

![Prometheus - Grafana](/node-red/prom-grafana.png)
