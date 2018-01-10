import traci
from traci import constants as tc
import plexe
import ccparams as cc

# lane change modes
DEFAULT_LC = 0b1001010101
DEFAULT_NOTRACI_LC = 0b1010101010
FIX_LC = 0b1000000000
FIX_LC_AGGRESSIVE = 0b0000000000


class PlexeImp(plexe.Plexe):
    """
    Plexe post sumo integration
    """

    def __init__(self):
        self.__versions = [
            "SUMO 0.31.0",
            "SUMO 0.32.0",
            "SUMO v0_31_0",
            "SUMO v0_32_0"
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
    def _set_lane_change_mode(vid, safe):
        if safe:
            traci.vehicle.setLaneChangeMode(vid, FIX_LC)
        else:
            traci.vehicle.setLaneChangeMode(vid, FIX_LC_AGGRESSIVE)

    def _change_lane(self, vid, current, direction, safe=True):
        if safe:
            self._set_lane_change_mode(vid, safe)
            traci.vehicle.changeLane(vid, current + direction, 0)
        else:
            state, state2 = traci.vehicle.getLaneChangeState(vid, direction)
            if state & tc.LCA_OVERLAPPING == 0:
                self._set_lane_change_mode(vid, safe)
                traci.vehicle.changeLane(vid, current + direction, 0)
                lane, safe, wait = self.lane_changes[vid]
                self.lane_changes[vid] = (lane, safe, True)

    def step(self, step):
        satisfied = []
        for vid, (lane, safe, wait) in self.lane_changes.iteritems():
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
                self._set_lane_change_mode(vid, safe)
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

    def set_fixed_acceleration(self, vid, activate, acceleration):
        self._set_par(vid, cc.PAR_FIXED_ACCELERATION,
                      cc.pack(1 if activate else 0, acceleration))

    def get_vehicle_data(self, vid):
        ret = self._get_par(vid, cc.PAR_SPEED_AND_ACCELERATION)
        return {plexe.U: ret[2], plexe.ACCELERATION: ret[1],
                plexe.SPEED: ret[0], plexe.POS_X: ret[3], plexe.POS_Y: ret[4],
                plexe.TIME: ret[5]}

    def get_crashed(self, vid):
        ret = self._get_single_par(vid, cc.PAR_CRASHED)
        return True if ret == 1 else False

    def get_radar_data(self, vid):
        ret = self._get_par(vid, cc.PAR_RADAR_DATA)
        return ret[0], ret[1]

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
        return {plexe.INDEX: ret[0], plexe.ACCELERATION: ret[2],
                plexe.SPEED: ret[1], plexe.POS_X: ret[3], plexe.POS_Y: ret[4],
                plexe.TIME: ret[5], plexe.LENGTH: ret[6]}

    def get_engine_data(self, vid):
        ret = self._get_par(vid, cc.PAR_ENGINE_DATA)
        return {plexe.GEAR: ret[0], plexe.RPM: ret[1]}

    def set_vehicle_data(self, vid, index, acceleration, speed, pos_x, pos_y,
                         time, length):
        self._set_par(vid, cc.CC_PAR_VEHICLE_DATA,
                      cc.pack(index, speed, acceleration, pos_x, pos_y, time,
                              length))

    def set_leader_vehicle_data(self, vid, acceleration, speed, pos_x, pos_y,
                                time):
        self._set_par(vid, cc.PAR_LEADER_SPEED_AND_ACCELERATION,
                      cc.pack(speed, acceleration, pos_x, pos_y, time))

    def set_front_vehicle_data(self, vid, acceleration, speed, pos_x, pos_y,
                               time):
        self._set_par(vid, cc.PAR_PRECEDING_SPEED_AND_ACCELERATION,
                      cc.pack(speed, acceleration, pos_x, pos_y, time))

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

    def set_leader_vehicle_fake_data(self, vid, acceleration, speed):
        self._set_par(vid, cc.PAR_LEADER_FAKE_DATA, cc.pack(speed,
                                                            acceleration))

    def set_front_vehicle_fake_data(self, vid, acceleration, speed, distance):
        self._set_par(vid, cc.PAR_FRONT_FAKE_DATA, cc.pack(speed,
                                                           acceleration,
                                                           distance))

    def set_acc_headway_time(self, vid, headway):
        self._set_par(vid, cc.PAR_ACC_HEADWAY_TIME, headway)
