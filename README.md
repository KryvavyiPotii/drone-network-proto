# drone-network

A crate that aims to model a UAV networks and the impact of electronic warfare and malware on them.

## Render image legend

* Green circle - command center transmission area.
* Yellow circle - destination point.
* Red circle - transmission area of an electronic warfare device that suppresses GPS signal.
  On contact a drone loses its global position and moves in the same horizontal direction in which it moved before the contact.
* Blue circle - transmission area of an electronic warfare device that suppresses control signal or an attacker device that spreads malware.

## Usage

```shell
$ drone_network --help
Models drone networks.

Usage: drone_network [OPTIONS]

Options:
  -c, --caption <plot caption>
          Set the plot caption [default: ]
      --width <plot width>
          Set the plot width [default: 400]
      --height <plot height>
          Set the plot height [default: 300]
      --time <simulation time>
          Set the simulation time [default: 15000]
  -e, --example <example number>
          Run an experiment by its number
  -x, --experiment <experiment title>
          Choose experiment title [possible values: delays, dos, control, gps, gpsspoof, infection, signals]
  -m, --network-model <network model>
          Choose network model [possible values: sl, sf]
  -d, --delay-multiplier <delay multiplier>
          Set signal transmission delay multiplier [default: 0.0]
  -t, --topology <network topology>
          Choose network topology [possible values: mesh, star]
      --display-delayless
          Show the same network model without delays as well ("delays" experiment)
      --dynamic
          Show signal colors when drones are moving ("signals" experiment)
  -i, --infection <infection type>
          Choose infection type ("infection" experiment) [possible values: indicator, jamming]
  -h, --help
          Print help
  -V, --version
          Print version
```
