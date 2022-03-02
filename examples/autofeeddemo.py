#!/usr/bin/env python
#
# Copyright (c) 2018-2022 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import os
import sys
import random

import math

from utils import add_platooning_vehicle, communicate, get_distance, \
    start_sumo, running

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
from plexe import Plexe, ACC, CACC, RPM, GEAR

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# cruising speed
SPEED = 120 / 3.6
# sinusoid frequency
FREQ = 0.2
# sinusoid amplitude
AMP = 10 / 3.6

# maneuver actors
LEADER = "v.0"
N_VEHICLES = 8


def add_vehicles(plexe, n, real_engine=False):
    """
    Adds a platoon of n vehicles to the simulation, plus an additional one
    farther away that wants to join the platoon
    :param plexe: API instance
    :param n: number of vehicles of the platoon
    :param real_engine: set to true to use the realistic engine model,
    false to use a first order lag model
    """
    # add a platoon of n vehicles
    for i in range(n):
        vid = "v.%d" % i
        add_platooning_vehicle(plexe, vid, (n - i + 1) * (DISTANCE + LENGTH)
                               + 50, 0, SPEED, DISTANCE, real_engine)
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
        else:
            plexe.set_active_controller(vid, CACC)
        if i > 0:
            plexe.enable_auto_feed(vid, True, LEADER, "v.%d" % (i - 1))


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    gui = True
    start_sumo("cfg/freeway.sumo.cfg", False, gui=gui)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    while running(demo_mode, step, 6000):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True, gui=gui)
            step = 0
            random.seed(1)

        traci.simulationStep()

        if step == 0:
            # create vehicles and track the joiner
            add_vehicles(plexe, N_VEHICLES, real_engine)
            if gui:
                traci.gui.trackVehicle("View #0", LEADER)
                traci.gui.setZoom("View #0", 20000)
        if step >= 1:
            if step % 10 == 0:
                time = step / 100.0
                speed = SPEED + AMP * math.sin(2 * math.pi * FREQ * time)
                plexe.set_cc_desired_speed(LEADER, speed)
        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1

    traci.close()


if __name__ == "__main__":
    main(False, False)
