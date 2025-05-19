# drone-network

A trait that aims to model a network of drones.

## Render legend

* Green zone - command center.
* Yellow zone - destination position.
* Red zone - GPS signal suppression.
  On contact a drone loses its global position and moves in the same horizontal direction in which he moved before the contact.
* Blue zone - control signal suppression.
  On contact a drone is disconnected from the swarm.

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
          Choose experiment title [possible values: delays, control, gps, gpsspoof, infection, signals]
  -m, --network-model <network model>
          Choose network model [possible values: ca, cn]
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
