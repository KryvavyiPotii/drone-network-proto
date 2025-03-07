# drone-network

A trait that aims to model a network of drones.

## Render legend

* Green zone - command center.
* Yellow zone - destination position.
* Red zone - GPS signal suppression.
  On contact a drone loses its global position and moves in the same horizontal direction in which he moved before the contact.
* Blue zone - control signal suppression.
  On contact a drone is disconnected from the swarm.
