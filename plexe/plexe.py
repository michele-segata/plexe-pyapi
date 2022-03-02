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
from os import environ, listdir
from os.path import join, splitext, dirname
from importlib import import_module
import sys
if 'SUMO_HOME' in environ:
    tools = join(environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

# available controllers
DRIVER = 0
ACC = 1
CACC = 2
FAKED_CACC = 3
PLOEG = 4
CONSENSUS = 5

U = "u"
ACCELERATION = "acceleration"
SPEED = "speed"
POS_X = "posx"
POS_Y = "posy"
TIME = "time"
INDEX = "index"
LENGTH = "length"
GEAR = "gear"
RPM = "rpm"
RADAR_DISTANCE = "rd"
RADAR_REL_SPEED = "rs"

ENGINE_MODEL_FOLM = 0x00
ENGINE_MODEL_REALISTIC = 0x01


class Plexe(traci.StepListener):

    latest = [1, 1, 0]
    versions = {
        'SUMO d1422e4780a': [0, 32, 0],
        'SUMO 619df188ac3': [0, 32, 0],
        'SUMO 1.0.1': [1, 0, 1],
        'SUMO 1.1.0': [1, 1, 0],
        'SUMO v1_1_0': [1, 1, 0]
    }

    def __init__(self):
        """
        Constructor. Instantiates methods' implementation depending on SUMO
        version. SUMO must be already started when instantiating the class
        """
        self.plexe = None
        api, version = traci.getVersion()
        if version in self.versions:
            self.version = self.versions[version]
        else:
            self.version = self.latest
            for k, v in self.versions.items():
                if version.startswith(k):
                    self.version = self.versions[k]
                    break

        files = listdir(join(dirname(__file__), "plexe_imp"))
        default_impl = None
        for f in files:
            name, ext = splitext(f)
            if ext == ".py":
                mod = import_module(".{}".format(name), "plexe.plexe_imp")
                try:
                    instance = mod.PlexeImp()
                except AttributeError:
                    continue
                versions = instance.register()
                for v in versions:
                    if version.startswith(v):
                        self.plexe = instance
                        break
                if 'default' in versions:
                    default_impl = instance
        if self.plexe is None:
            self.plexe = default_impl
        if self.plexe is None:
            print("No Plexe API implementation found for %s" % version)
            raise Exception()

    def step(self, step):
        self.plexe.step(step)
        return True

    def set_cc_desired_speed(self, vid, speed):
        """
        Sets the cruise control desired speed
        :param vid: vehicle id
        :param speed: speed in m/s
        """
        self.plexe.set_cc_desired_speed(vid, speed)

    def set_active_controller(self, vid, controller):
        """
        Sets the currently active controller
        :param vid: vehicle id
        :param controller: DRIVER = 0, ACC = 1, CACC = 2, FAKED_CACC = 3,
        PLOEG = 4, CONSENSUS = 5
        """
        self.plexe.set_active_controller(vid, controller)

    def set_fixed_lane(self, vid, lane, safe=True):
        """
        Tell a vehicle to move and stay on a specific lane
        :param vid: vehicle id
        :param lane: lane index (0-based)
        :param safe: if true, respect safety distance, if false change lane
        as soon as possible
        """
        self.plexe.set_fixed_lane(vid, lane, safe)

    def disable_fixed_lane(self, vid):
        """
        Disables the fixed lane set with set_fixed_lane and gives lane change
        decisions back to the lane change model
        :param vid: vehicle id
        """
        self.plexe.disable_fixed_lane(vid)

    def set_fixed_acceleration(self, vid, activate, acceleration):
        """
        Tell a vehicle to apply a certain acceleration, or to switch back to
        standard behavior
        :param vid: vehicle id
        :param activate: if true, sets a constant acceleration profile to the
        vehicle, if false, gives the control back to the control algorithms
        :param acceleration: constant acceleration to apply
        """
        self.plexe.set_fixed_acceleration(vid, activate, acceleration)

    def get_vehicle_data(self, vid):
        """
        Returns vehicle dynamics data of an automated vehicle
        :param vid: vehicle id
        :return: a VehicleData object
        """
        return self.plexe.get_vehicle_data(vid)

    def get_crashed(self, vid):
        """
        Returns whether an automated vehicle crashed or not
        :param vid: vehicle id
        :return: True if crashed, False otherwise
        """
        return self.plexe.get_crashed(vid)

    def get_radar_data(self, vid):
        """
        Returns data measured by radar, i.e., distance and relative speed to
        the front vehicle. If there is no front vehicle or it is too far (
        more than 250 meters) then the returned distance is set to -1
        :param vid: vehicle id
        :return: a dictionary including plexe.RADAR_DISTANCE and
        plexe.RADAR_REL_SPEED keys
        """
        return self.plexe.get_radar_data(vid)

    def get_lanes_count(self, vid):
        """
        Returns the number of lanes of the road the vehicle is currently
        traveling on
        :param vid: vehicle id
        :return: the number of lanes of the current road
        """
        return self.plexe.get_lanes_count(vid)

    def get_distance_to_end(self, vid):
        """
        Returns the distance to the end of the route
        :param vid: vehicle id
        :return: the distance to route end in meters
        """
        return self.plexe.get_distance_to_end(vid)

    def get_distance_from_begin(self, vid):
        """
        Returns the distance from the beginning of the route
        :param vid: vehicle id
        :return: the distance from route beginning in meters
        """
        return self.plexe.get_distance_from_begin(vid)

    def get_active_controller(self, vid):
        """
        Returns the active car controller
        :param vid: vehicle id
        :return: active controller, DRIVER = 0, ACC = 1, CACC = 2,
        FAKED_CACC = 3, PLOEG = 4, CONSENSUS = 5
        """
        return self.plexe.get_active_controller(vid)

    def get_acc_acceleration(self, vid):
        """
        Returns the acceleration computed by the ACC, which is computed even
        when the ACC is not currently controlling the car
        :param vid: vehicle id
        :return: computed ACC acceleration in m/s^2
        """
        return self.plexe.get_acc_acceleration(vid)

    def get_cacc_spacing(self, vid):
        """
        Returns the fixed spacing for the PATH CACC controller
        :param vid: vehicle id
        :return: spacing in meters
        """
        return self.plexe.get_cacc_spacing(vid)

    def get_stored_vehicle_data(self, vid, other_vid):
        """
        Returns the data stored by this vehicle about another vehicle
        :param vid: vehicle id
        :param other_vid: index of the vehicle in the same platoon, NOT the
        sumo vehicle id
        :return: a VehicleData object. If the given index is greater or equal
        than the platoon size, then the index will be set to -1
        """
        return self.plexe.get_stored_vehicle_data(vid, other_vid)

    def get_engine_data(self, vid):
        """
        If the vehicle is using the realistic engine model, this method
        returns the current gear and the engine RPM
        :param vid: vehicle id
        :return: a dictionary including the plexe.GEAR and plexe.RPM keys. If
        the vehicle is not using the realistic engine model, the gear is set
        to -1
        """
        return self.plexe.get_engine_data(vid)

    def set_vehicle_data(self, vid, vehicle_data):
        """
        Sets information about a vehicle in the platoon. This is currently
        only used by the CONSENSUS controller. See also
        set_leader_vehicle_data and set_front_vehicle_data.
        :param vid: vehicle id, i.e., the vehicle that will store the
        information
        :param vehicle_data: a VehicleData object
        """
        return self.plexe.set_vehicle_data(vid, vehicle_data)

    def set_leader_vehicle_data(self, vid, vehicle_data):
        """
        Sets data about the platoon leader
        :param vid: vehicle which stores the information
        :param vehicle_data: a VehicleData object
        """
        return self.plexe.set_leader_vehicle_data(vid, vehicle_data)

    def set_front_vehicle_data(self, vid, vehicle_data):
        """
        Sets data about the front vehicle
        :param vid: vehicle which stores the information
        :param vehicle_data: a VehicleData object
        """
        return self.plexe.set_front_vehicle_data(vid, vehicle_data)

    def set_vehicle_position(self, vid, position):
        """
        Sets the position of the vehicle within its platoon
        :param vid: vehicle id
        :param position: 0-based position index
        """
        return self.plexe.set_vehicle_position(vid, position)

    def set_platoon_size(self, vid, size):
        """
        Sets the number of vehicles for the platoon a vehicle belongs to
        :param vid: vehicle id
        :param size: number of vehicles
        """
        return self.plexe.set_platoon_size(vid, size)

    def set_path_cacc_parameters(self, vid, distance=None, xi=None,
                                 omega_n=None, c1=None):
        """
        Sets the parameters for the PATH CACC. If a parameter is set to None,
        it won't be set and it will keep its current value
        :param vid: vehicle id
        :param distance: constant spacing in meters
        :param xi: damping ratio
        :param omega_n: bandwidth
        :param c1: leader data weighting parameter
        """
        return self.plexe.set_path_cacc_parameters(vid, distance, xi,
                                                   omega_n, c1)

    def set_ploeg_cacc_parameters(self, vid, k_p=None, k_d=None, headway=None):
        """
        Sets the parameters for the PLOEG's CACC. If a parameter is set to
        None, it won't be set and it will keep its current value
        :param vid: vehicle id
        :param k_p: proportional gain
        :param k_d: derivative gain
        :param headway: time headway in seconds
        """
        return self.plexe.set_ploeg_cacc_parameters(vid, k_p, k_d, headway)

    def set_engine_tau(self, vid, tau):
        """
        Sets the engine time constant for the first order lag engine model
        :param vid: vehicle id
        :param tau: time constant in seconds
        """
        return self.plexe.set_engine_tau(vid, tau)

    def set_engine_model(self, vid, model):
        """
        Sets the engine model for the given vehicle
        :param vid: vehicle id
        :param model: engine model id. Possible values are ENGINE_MODEL_FOLM
        and ENGINE_MODEL_REALISTIC
        """
        return self.plexe.set_engine_model(vid, model)

    def set_vehicle_model(self, vid, model):
        """
        Sets the vehicle model when choosing the realistic engine model.
        Basically, this method chooses the real vehicle characteristics
        :param vid: vehicle id
        :param model: model id as defined in the vehicles.xml file
        """
        return self.plexe.set_vehicle_model(vid, model)

    def set_vehicles_file(self, vid, filename):
        """
        Sets the xml file from which the realistic engine model should load
        vehicles characteristics
        :param vid: vehicle id
        :param filename: xml file
        """
        return self.plexe.set_vehicles_file(vid, filename)

    def set_leader_vehicle_fake_data(self, vid, vehicle_data):
        """
        Sets the leader vehicle data for the FAKED CACC controller
        :param vid: vehicle id
        :param vehicle_data: a VehicleData object with u, acceleration,
        and speed fields set
        """
        return self.plexe.set_leader_vehicle_fake_data(vid, vehicle_data)

    def set_front_vehicle_fake_data(self, vid, vehicle_data, distance):
        """
        Sets the front vehicle data for the FAKED CACC controller
        :param vid: vehicle id
        :param vehicle_data: a VehicleData object with u, acceleration,
        and speed fields set
        :param distance: distance to front vehicle in m
        """
        return self.plexe.set_front_vehicle_fake_data(vid, vehicle_data,
                                                      distance)

    def set_acc_headway_time(self, vid, headway):
        """
        Sets the headway time for the ACC
        :param vid: vehicle id
        :param headway: headway in seconds
        """
        return self.plexe.set_acc_headway_time(vid, headway)

    def use_controller_acceleration(self, vid, use):
        """
        Determines whether PATH's and PLOEG's CACCs should use the controller
        or the real acceleration when computing the control action
        :param vid: vehicle id
        :param use: if set to true, the vehicle will use the controller
        acceleration
        """
        return self.plexe.use_controller_acceleration(vid, use)

    def enable_auto_feed(self, vid, enable, leader_id=None, front_id=None):
        """
        Activates or deactivates autofeeding, meaning that the user is not
        simulating inter-vehicle communication, so the CACCs will
        automatically take the required data from other vehicles automatically
        :param vid: vehicle id
        :param enable: boolean to enable or disable auto feeding
        :param leader_id: id of the leader vehicle. When disabling auto
        feeding, this parameter can be omitted
        :param front_id: id of the front vehicle. When disabling auto
        feeding, this parameter can be omitted
        """
        if enable and (leader_id is None or front_id is None):
            return False
        return self.plexe.enable_auto_feed(vid, enable, leader_id, front_id)

    def use_prediction(self, vid, enable):
        """
        Activates or deactivates prediction, i.e., interpolation of missing
        data for the control system
        :param vid: vehicle id
        :param enable: boolean to enable or disable prediction
        """
        return self.plexe.use_prediction(vid, enable)

    def add_member(self, vid, member_id, position):
        """
        Adds a platoon member to this vehicle, usually considered to be the
        leader. Members are used to perform coordinated, whole-platoon lane
        changes
        :param vid: vehicle id
        :param member_id: sumo id of the member being added
        :param position: position (0-based) of the vehicle
        """
        return self.plexe.add_member(vid, member_id, position)

    def remove_member(self, vid, member_id):
        """
        Removes a platoon member from this vehicle, usually considered to be
        the leader. Members are used to perform coordinated, whole-platoon
        lane changes
        :param vid: vehicle id
        :param member_id: sumo id of the member being removed
        """
        return self.plexe.remove_member(vid, member_id)

    def enable_auto_lane_changing(self, vid, enable):
        """
        Enables/disables automatic, coordinated, whole-platoon lane changes.
        This function should be invoked on the leader which decides whether
        the platoon can gain speed by changing lane. The leader will then
        check whether lane changing is possible and, in case, do so
        :param vid: vehicle id
        :param enable: enable or disable automatic platoon lane changes
        """
        return self.plexe.enable_auto_lane_changing(vid, enable)
