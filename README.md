# drone-network

A crate that aims to model a UAV networks and the impact of electronic warfare and malware on them.

## Render image legend

* **Green circle** - command center transmission area.
* **Yellow circle** - destination point.
* **Orange circle** - transmission area of an attacker device that executes GPS spoofing attack.
* **Red circle** - transmission area of an electronic warfare device that suppresses GPS signal.
  On contact a drone loses its global position and moves in the same horizontal direction in which it moved before the contact.
* **Pink circle** - transmission area of an attacker device that spreads malware.
* **Blue circle** - transmission area of an electronic warfare device that suppresses control signal.

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
  -x, --experiment <experiment title>
          Choose experiment title [possible values: delays, control, gps, gpsspoof, malware, signals, signalloss]
      --trx <trx system>
          Choose device TRX system type [default: both] [possible values: both, color, strength]
  -t, --topology <network topology>
          Choose network topology [default: both] [possible values: both, mesh, star]
  -n <drone count>
          Set the number of drones in the network [default: 100]
  -d, --delay-multiplier <delay multiplier>
          Set signal transmission delay multiplier [default: 0.0]
      --display-delayless
          Show the same network model without delays as well ("delays" experiment)
      --display-propagation
          Show malware propagation as well ("malware" experiment)
  -i, --infection <malware type>
          Choose infection type ("malware" experiment) [possible values: dos, indicator, jamming]
  -h, --help
          Print help
  -V, --version
          Print version
```
