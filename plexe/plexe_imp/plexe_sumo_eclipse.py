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
import traci
from traci import constants as tc
import plexe
from plexe.plexe_imp import ccparams as cc
from plexe.vehicle_data import VehicleData

# lane change modes
DEFAULT_LC = 0b011001010101
DEFAULT_NOTRACI_LC = 0b1010101010
FIX_LC = 0b1000000000
FIX_LC_AGGRESSIVE = 0b0000000000


class PlexeImp(plexe.Plexe):
    """
    Plexe post sumo integration
    """

    def __init__(self):
        self.__versions = [
            'default',
            'SUMO d1422e4780a',
            'SUMO 619df188ac3',
            'SUMO 1.0.1',
            'SUMO 1.1.0',
            'SUMO v1_1_0',
        ]
        self.lane_changes = {}

    def register(self):
        return self.__versions

    @staticmethod
    def _set_par(vid, par, value):
        """
        Shorthand for the setParameter method
        :param vid: vehicle id
        :param par: parameter name
        :param value: numeric or string value for the parameter
        """
        traci.vehicle.setParameter(vid, "carFollowModel.%s" % par, str(value))

    @staticmethod
    def _get_par(vid, par, *args):
        """
        Shorthand for the getParameter method
        :param vid: vehicle id
        :param par: parameter name
        :param args: optional arguments
        :return: the required parameter value
        """
        arguments = cc.pack(par, *args)
        ret = traci.vehicle.getParameter(vid, "carFollowModel." + arguments)
        return cc.unpack(ret)

    @staticmethod
    def _get_single_par(vid, par, *args):
        """
        Shorthand for getting the value of a parameter which has only a
        single return value
        :param vid: vehicle id
        :param par: parameter name
        :param args: optional arguments
        :return: the required parameter value
        """
        ret = PlexeImp._get_par(vid, par, *args)
        return ret[0]

    @staticmethod
    def _set_lane_change_mode(vid, safe, fixed):
        if not fixed:
            traci.vehicle.setLaneChangeMode(vid, DEFAULT_LC)
        else:
            if safe:
                traci.vehicle.setLaneChangeMode(vid, FIX_LC)
            else:
                traci.vehicle.setLaneChangeMode(vid, FIX_LC_AGGRESSIVE)

    def _change_lane(self, vid, current, direction, safe=True):
        if safe:
            self._set_lane_change_mode(vid, safe, True)
            traci.vehicle.changeLane(vid, current + direction, 0)
        else:
            state, state2 = traci.vehicle.getLaneChangeState(vid, direction)
            if state & tc.LCA_OVERLAPPING == 0:
                self._set_lane_change_mode(vid, safe, True)
                traci.vehicle.changeLane(vid, current + direction, 0)
                lane, safe, wait = self.lane_changes[vid]
                self.lane_changes[vid] = (lane, safe, True)

    def step(self, step):
        satisfied = []
        for vid, (lane, safe, wait) in self.lane_changes.items():
            if wait:
                lane, safe, wait = self.lane_changes[vid]
                self.lane_changes[vid] = (lane, safe, False)
                continue
            current = traci.vehicle.getLaneIndex(vid)
            n_lanes = lane - current
            if n_lanes > 0:
                direction = 1
            elif n_lanes < 0:
                direction = -1
            else:
                direction = 0
            if direction == 0:
                satisfied.append(vid)
                self._set_lane_change_mode(vid, safe, True)
            else:
                self._change_lane(vid, current, direction, safe)
        for vid in satisfied:
            del self.lane_changes[vid]

    def set_cc_desired_speed(self, vid, speed):
        self._set_par(vid, cc.PAR_CC_DESIRED_SPEED, speed)

    def set_active_controller(self, vid, controller):
        self._set_par(vid, cc.PAR_ACTIVE_CONTROLLER, controller)

    def set_fixed_lane(self, vid, lane, safe=True):
        self.lane_changes[vid] = (lane, safe, False)

    def disable_fixed_lane(self, vid):
        if vid in self.lane_changes.keys():
            del self.lane_changes[vid]
        self._set_lane_change_mode(vid, False, False)

    def set_fixed_acceleration(self, vid, activate, acceleration):
        self._set_par(vid, cc.PAR_FIXED_ACCELERATION,
                      cc.pack(1 if activate else 0, acceleration))

    def get_vehicle_data(self, vid):
        ret = self._get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
        return VehicleData(None, ret[2], ret[1], ret[0], ret[3], ret[4], ret[5])

    def get_crashed(self, vid):
        ret = self._get_single_par(vid, cc.PAR_CRASHED)
        return True if ret == 1 else False

    def get_radar_data(self, vid):
        ret = self._get_par(vid, cc.PAR_RADAR_DATA)
        return {plexe.RADAR_DISTANCE: ret[0], plexe.RADAR_REL_SPEED: ret[1]}

    def get_lanes_count(self, vid):
        return self._get_single_par(vid, cc.PAR_LANES_COUNT)

    def get_distance_to_end(self, vid):
        return self._get_single_par(vid, cc.PAR_DISTANCE_TO_END)

    def get_distance_from_begin(self, vid):
        return self._get_single_par(vid, cc.PAR_DISTANCE_FROM_BEGIN)

    def get_active_controller(self, vid):
        return self._get_single_par(vid, cc.PAR_ACTIVE_CONTROLLER)

    def get_acc_acceleration(self, vid):
        return self._get_single_par(vid, cc.PAR_ACC_ACCELERATION)

    def get_cacc_spacing(self, vid):
        return self._get_single_par(vid, cc.PAR_CACC_SPACING)

    def get_stored_vehicle_data(self, vid, other_vid):
        ret = self._get_par(vid, cc.CC_PAR_VEHICLE_DATA, other_vid)
        return VehicleData(ret[0], ret[7], ret[2], ret[1], ret[3], ret[4],
                           ret[5], ret[6])

    def get_engine_data(self, vid):
        ret = self._get_par(vid, cc.PAR_ENGINE_DATA)
        return {plexe.GEAR: ret[0], plexe.RPM: ret[1]}

    def set_vehicle_data(self, vid, vehicle_data):
        self._set_par(vid, cc.CC_PAR_VEHICLE_DATA,
                      cc.pack(vehicle_data.index, vehicle_data.speed,
                              vehicle_data.acceleration, vehicle_data.pos_x,
                              vehicle_data.pos_y, vehicle_data.time,
                              vehicle_data.length, vehicle_data.u))

    def set_leader_vehicle_data(self, vid, vehicle_data):
        self._set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION,
                      cc.pack(vehicle_data.speed, vehicle_data.acceleration,
                              vehicle_data.pos_x, vehicle_data.pos_y,
                              vehicle_data.time, vehicle_data.u))

    def set_front_vehicle_data(self, vid, vehicle_data):
        self._set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION,
                      cc.pack(vehicle_data.speed, vehicle_data.acceleration,
                              vehicle_data.pos_x, vehicle_data.pos_y,
                              vehicle_data.time, vehicle_data.u))

    def set_vehicle_position(self, vid, position):
        self._set_par(vid, cc.CC_PAR_VEHICLE_POSITION, position)

    def set_platoon_size(self, vid, size):
        self._set_par(vid, cc.CC_PAR_PLATOON_SIZE, size)

    def set_path_cacc_parameters(self, vid, distance=None, xi=None,
                                 omega_n=None, c1=None):
        if distance is not None:
            self._set_par(vid, cc.PAR_CACC_SPACING, distance)
        if xi is not None:
            self._set_par(vid, cc.CC_PAR_CACC_XI, xi)
        if omega_n is not None:
            self._set_par(vid, cc.CC_PAR_CACC_OMEGA_N, omega_n)
        if c1 is not None:
            self._set_par(vid, cc.CC_PAR_CACC_C1, c1)

    def set_ploeg_cacc_parameters(self, vid, k_p=None, k_d=None, headway=None):
        if k_p is not None:
            self._set_par(vid, cc.CC_PAR_PLOEG_KP, k_p)
        if k_d is not None:
            self._set_par(vid, cc.CC_PAR_PLOEG_KD, k_d)
        if headway is not None:
            self._set_par(vid, cc.CC_PAR_PLOEG_H, headway)

    def set_engine_tau(self, vid, tau):
        self._set_par(vid, cc.CC_PAR_ENGINE_TAU, tau)

    def set_engine_model(self, vid, model):
        if model not in [cc.CC_ENGINE_MODEL_FOLM,
                         cc.CC_ENGINE_MODEL_REALISTIC]:
            return False
        self._set_par(vid, cc.CC_PAR_VEHICLE_ENGINE_MODEL, model)

    def set_vehicle_model(self, vid, model):
        self._set_par(vid, cc.CC_PAR_VEHICLE_MODEL, model)

    def set_vehicles_file(self, vid, filename):
        self._set_par(vid, cc.CC_PAR_VEHICLES_FILE, filename)

    def set_leader_vehicle_fake_data(self, vid, vehicle_data):
        self._set_par(vid, cc.PAR_LEADER_FAKE_DATA,
                      cc.pack(vehicle_data.speed, vehicle_data.acceleration,
                              vehicle_data.u))

    def set_front_vehicle_fake_data(self, vid, vehicle_data, distance):
        self._set_par(vid, cc.PAR_FRONT_FAKE_DATA,
                      cc.pack(vehicle_data.speed, vehicle_data.acceleration,
                              distance, vehicle_data.u))

    def set_acc_headway_time(self, vid, headway):
        self._set_par(vid, cc.PAR_ACC_HEADWAY_TIME, headway)

    def use_controller_acceleration(self, vid, use):
        self._set_par(vid, cc.PAR_USE_CONTROLLER_ACCELERATION, 1 if use else 0)

    def enable_auto_feed(self, vid, enable, leader_id=None, front_id=None):
        if enable:
            self._set_par(vid, cc.PAR_USE_AUTO_FEEDING,
                          cc.pack(1, leader_id, front_id))
        else:
            self._set_par(vid, cc.PAR_USE_AUTO_FEEDING, 0)

    def use_prediction(self, vid, enable):
        self._set_par(vid, cc.PAR_USE_PREDICTION, 1 if enable else 0)

    def add_member(self, vid, member_id, position):
        self._set_par(vid, cc.PAR_ADD_MEMBER, cc.pack(member_id, position))

    def remove_member(self, vid, member_id):
        self._set_par(vid, cc.PAR_ADD_MEMBER, member_id)

    def enable_auto_lane_changing(self, vid, enable):
        self._set_par(vid, cc.PAR_ENABLE_AUTO_LANE_CHANGE, 1 if enable else 0)
