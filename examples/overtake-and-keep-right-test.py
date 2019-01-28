#!/usr/bin/env python

import os
import random
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci
from plexe import Plexe, ACC, GEAR, RPM
from utils import start_sumo, running, get_status, add_platooning_vehicle, \
    add_vehicle


def main(demo_mode, real_engine, setter=None):

    start_sumo("cfg/freeway.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    state_left = None
    state_right = None
    random.seed(1)
    while running(demo_mode, step, 6000):

        if demo_mode and step == 6000:
            start_sumo("cfg/freeway.sumo.cfg", True)
            step = 0
            random.seed(1)

        traci.simulationStep()

        if step == 1:
            add_platooning_vehicle(plexe, "p0", 150, 0, 25, 5, real_engine)
            add_vehicle(plexe, "v0", 140, 1, 25, "passenger")
            add_vehicle(plexe, "v1", 250, 0, 20, "passenger2")
            traci.gui.trackVehicle("View #0", "p0")
            traci.gui.setZoom("View #0", 50000)
            plexe.set_active_controller("p0", ACC)
            plexe.set_cc_desired_speed("p0", 25)
            traci.vehicle.setSpeedMode("p0", 0)

        if step > 1:
            state = traci.vehicle.getLaneChangeState("p0", 1)[0]
            if state_left != state:
                state_left = state
                str_status = get_status(state)
                print("Step %d, vehicle p0. Lane change status (LEFT ): %s" %
                      (step, str_status))
            state = traci.vehicle.getLaneChangeState("p0", -1)[0]
            if state_right != state:
                state_right = state
                str_status = get_status(state)
                print("Step %d, vehicle p0. Lane change status (RIGHT): %s" %
                      (step, str_status))

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
    main(True, False)
