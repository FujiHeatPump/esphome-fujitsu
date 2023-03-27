# Intro
This project uses Fujitsu's proprietary protocol to control a Fujitsu AC(heat pump) unit, interfacing with Home Assistant through ESPHome.

This project is entirely based on unreality [FujiHeatPump](https://github.com/unreality/FujiHeatPump) project.
Huge thanks to [unreality](https://github.com/unreality/) and [jaroslawprzybylowicz](https://github.com/jaroslawprzybylowicz/fuji-iot)!

# How to use:
## Hardware:

See [FujiHeatPump](https://github.com/unreality/FujiHeatPump)'s readme file, I use that exact circuit with a ESP32 development board.
IMPORTANT: I connected the MCP2025 TX RX to the ESP32's Serial 2 line, I used the Serial 1 line for debugging.

## ESPHome:


See fujitsu.yaml for a sample config file.
Then run your esphome (tested with esphome version 2022.2.3) compile or upload command:
```bash
$ esphome compile fujitsu.yaml
```

## Default behaviour:

* The MCP2025 is connected to Serial 2
* The controller is registered as secondary controller, see `void FujitsuClimate::setup()` in `FujitsuClimate.cpp`
* My AC works on 1 degree C step, 16-30 degree temperature range etc...
see `void climate::ClimateTraits FujitsuClimate::traits()` in `FujitsuClimate.cpp` for more details,
modify the function to suit your unit.

## Known issues:
* Sometimes a command is sent but somehow it fails to control the unit, I would have to resend the command from Home Assistant
* Setting ECO mode from Home Assistant does not work, but receiving ECO mode change from the controller is fine

## Planned work:
* Get this project merged into the main ESPHome repository

## How to contirbute:
* Use github issues to start a discussion
* Create pull requests
