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
from utils import add_platooning_vehicle, start_sumo, running, communicate

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from plexe import Plexe, ACC, CACC, RPM, GEAR, RADAR_DISTANCE

# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5
# cruising speed
SPEED = 120/3.6
# distance between multiple platoons
PLATOON_DISTANCE = SPEED * 1.5 + 2
# vehicle who starts to brake
BRAKING_VEHICLE = "v.0.0"


def add_vehicles(plexe, n, n_platoons, real_engine=False):
    """
    Adds a set of platoons of n vehicles each to the simulation
    :param plexe: API instance
    :param n: number of vehicles of the platoon
    :param n_platoons: number of platoons
    :param real_engine: set to true to use the realistic engine model,
    false to use a first order lag model
    :return: returns the topology of the platoon, i.e., a dictionary which
    indicates, for each vehicle, who is its leader and who is its front
    vehicle. The topology can the be used by the data exchange logic to
    automatically fetch data from leading and front vehicle to feed the CACC
    """
    # add a platoon of n vehicles
    topology = {}
    p_length = n * LENGTH + (n - 1) * DISTANCE
    for p in range(n_platoons):
        for i in range(n):
            vid = "v.%d.%d" % (p, i)
            add_platooning_vehicle(plexe, vid, (n-p+1) *
                                   (p_length + PLATOON_DISTANCE) + (n-i+1) *
                                   (DISTANCE+LENGTH), 0, SPEED, DISTANCE,
                                   real_engine)
            plexe.set_fixed_lane(vid, 0, False)
            traci.vehicle.setSpeedMode(vid, 0)
            plexe.use_controller_acceleration(vid, False)
            if i == 0:
                plexe.set_active_controller(vid, ACC)
            else:
                plexe.set_active_controller(vid, CACC)
            if i > 0:
                topology[vid] = {"front": "v.%d.%d" % (p, i - 1),
                                 "leader": "v.%d.0" % p}
            else:
                topology[vid] = {}
    return topology


def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    topology = dict()
    min_dist = 1e6

    while running(demo_mode, step, 1500):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 1500:
            print("Min dist: %f" % min_dist)
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            random.seed(1)

        traci.simulationStep()

        if step == 0:
            # create vehicles and track the braking vehicle
            topology = add_vehicles(plexe, 8, 1, real_engine)
            traci.gui.trackVehicle("View #0", BRAKING_VEHICLE)
            traci.gui.setZoom("View #0", 20000)
        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology)
        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)
        if step == 500:
            plexe.set_fixed_acceleration(BRAKING_VEHICLE, True, -6)
        if step > 1:
            radar = plexe.get_radar_data("v.0.1")
            if radar[RADAR_DISTANCE] < min_dist:
                min_dist = radar[RADAR_DISTANCE]

        step += 1

    traci.close()


if __name__ == "__main__":
    main(True, False)
