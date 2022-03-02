#!/usr/bin/env python
#
# Copyright (C) 2018-2022 Michele Segata <msegata@disi.unitn.it>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301, USA.
#

import os
import sys
import random
from utils import add_platooning_vehicle, start_sumo, running

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
import sumolib
from plexe import Plexe, ACC, GEAR, RPM, ENGINE_MODEL_REALISTIC

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "-c", "cfg/freeway.sumo.cfg"]


# vehicles to be inserted
VEHICLES = ["alfa-147", "audi-r8", "bugatti-veyron"]
TRACK = 1


def add_vehicles(plexe):
    """
    Adds the vehicles to the simulation
    """
    for i in range(len(VEHICLES)):
        vid = VEHICLES[i]
        add_platooning_vehicle(plexe, vid, 10, i, 0, 5)
        plexe.set_fixed_lane(vid, i, False)
        plexe.set_active_controller(vid, ACC)
        plexe.set_engine_model(vid, ENGINE_MODEL_REALISTIC)
        plexe.set_vehicles_file(vid, "vehicles.xml")
        plexe.set_vehicle_model(vid, vid)
        plexe.set_fixed_acceleration(vid, True, 0)
        traci.vehicle.setSpeedMode(vid, 0)


def main(demo_mode, real_engine=True, setter=None):
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    step = 0
    plexe = Plexe()
    traci.addStepListener(plexe)

    while running(demo_mode, step, 4000):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 4000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            random.seed(1)

        traci.simulationStep()
        if step == 0:
            # create vehicles and track one of them
            add_vehicles(plexe)
            traci.gui.trackVehicle("View #0", VEHICLES[TRACK])
            traci.gui.setZoom("View #0", 20000)
        if step == 100:
            # at 1 second let them accelerate as much as they can
            for vid in VEHICLES:
                plexe.set_fixed_acceleration(vid, True, 20)

        # in the dashboard, show the engine information about the vehicle
        # being tracked
        tracked_id = traci.gui.getTrackedVehicle("View #0")
        if setter is not None and tracked_id != "":
            ed = plexe.get_engine_data(tracked_id)
            vd = plexe.get_vehicle_data(tracked_id)
            setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, True)
