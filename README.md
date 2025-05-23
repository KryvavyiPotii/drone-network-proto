# drone-network

A crate that aims to model a UAV networks and the impact of electronic warfare and malware on them.

## Render image legend

* Green circle - command center transmission area.
* Yellow circle - destination point.
* Red circle - GPS electronic warfare device transmission area.
  On contact a drone loses its global position and moves in the same horizontal direction in which it moved before the contact.
* Blue circle - transmission area of electronic warfare device that suppresses control signal.

## Usage

```shell
$ drone_network --help
Models drone networks.

Usage: drone_network [OPTIONS]

Options:
  -c, --caption <plot caption>
          Set the plot caption [default: ]
  -e, --example <example number>
          Run an experiment by its number
  -x, --experiment <experiment title>
          Choose experiment title [possible values: delays, dos, control, gps, gpsspoof, infection, signals]
  -m, --network-model <network model>
          Choose network model [possible values: sl, sf]
  -d, --delay-multiplier <delay multiplier>
          Set communication delay for complex network model [default: 0.0]
  -t, --topology <network topology>
          Choose network topology [possible values: mesh, star]
      --display-delayless
          Show the same network model without delays as well
      --dynamic
          Show signal colors when drones are moving
  -i, --infection <infection type>
          Choose infection type [possible values: indicator, jamming]
  -h, --help
          Print help
  -V, --version
          Print version
```
