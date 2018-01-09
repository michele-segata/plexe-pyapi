Plexe APIs for Python
=====================

This repository includes the APIs for the platooning functions provided
by Plexe in SUMO.

Installation
------------

Clone the repository and then install the APIs using `pip`:
```
cd plexe-pyapi
sudo pip install .
```
To uninstall them:
```
sudo pip uninstall PlexeAPI
```

Usage
-----

The APIs are provided by a single class (`plexe.Plexe`) which should be
added as a `StepListener` to `traci`. Look at the following snippet for
an example:
```python
from plexe import Plexe, ACC
import traci

sumo_cmd = ["sumo-gui", "-c", "sumo.cfg"]
traci.start(sumo_cmd)
plexe = Plexe()
traci.addStepListener(plexe)

traci.simulationStep()
plexe.set_active_controller("vehicle.0", ACC)
plexe.set_cc_desired_speed("vehicle.0", 30)
plexe.set_fixed_lane("vehicle.0", 0)
```
