import os
import sys
import ccparams as cc
import utils
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

# Lane Change Modes
FIX_LC = 0b0000000000
COOP_LC = 0b011001001000
DEFAULT_LC = 0b1001010101
REMOVE_PARKING = 0x01

class Plane:
    """
    Planes for platoonable lanes, this class manages the lanes of the edges in a highway, every lane in the highway
    formats vehicles into conforming with its speed and spacing requirements thereby forcing the vehicles to move 
    platoons.
    """
    # Desired gap
    DISTANCE = 5
    # Vehicle length
    VLENGTH = 4
    # Platoon length
    N_VEHICLES = 24
    # Cruising speed
    SPEED = 35
    # Minimum platooning Speed
    PSPEED = 48
    # Initial state of planes
    NON = 0
    # Plane has changed lanes
    SWITCHED = 1
    # Plane is passing a lane-change flag
    PASSING_FLAG = 2
    # Plane has passed a lane-change flag
    FLAG_PASSED = 3
    # Points of interest representing lane change stations
    pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
    # Registry of times the platoon leader has found flags
    time_flags_found = [[0],[0],[0],[0]]
    def __init__(self,laneID, vehicles, lane_spacing, lane_speed, platoonable = False):
        """
        There are four lanes per edge in the setup for the simulation, each plane is a set of vehicles on a specific lane and edge 
        of the highway. It has an ID, fixed length, its allowed speed and spacing requirements.
        """
        self.states = [self.NON] # Set the initial state of the plane to NON
        self._ID = laneID # The plane as the ID of the lane
        self._members = vehicles # all vehicles of the plane at each timestep
        self._father_vehicle = vehicles[0] # the leader of the first platoon
        self._children_vehicles = [vehicle for vehicle in self._members if self._members.index(vehicle) != 0 and self._members.index(vehicle) % self.N_VEHICLES == 0] # leaders of subsequent vehicles
        self._grandchildren_vehicles =  [vehicle for vehicle in self._members if self._members.index(vehicle) != 0 and self._members.index(vehicle) % self.N_VEHICLES != 0]# followers of platoon leaders

    def plane_members(self):
        """
        Returns list of all vehicles in this plane
        """
        members = self._members
        return members

    def plane_leader(self):
        """
        Returns the ID of the lead vehicle
        """
        leader = self._father_vehicle
        return leader

    def plane_sec_leaders(self):
        """
        Returns the list of all secondary leaders
        """
        sec_leaders = self._children_vehicles
        return sec_leaders

    def plane_followers(self):
        """Returns the list of all follower vehicles"""
        followers = self._grandchildren_vehicles
        return followers

    def veh_pos_pairs(self, lane):
        """ 
        Returns a sorted dictionary of key: value pairs of vehicles and
        their positions on the lane
        """
        lanemembership = {self._ID: self.plane_followers()}
        locdic = {}
        sortedlocdict={}
        i=0
        for vehicle in self._members:
            try:
                loc = traci.vehicle.getLanePosition(vehicle)
            except:
                print("Veh no longer in simulation")

            else:
                locdic.update({vehicle:loc})
                sortedlocdict = sorted(locdic.items(), key=lambda kv: kv[1])
        return sortedlocdict

    def topo_contsructor(self):
        """
        Based on the vehicle-position pairs creates the topology of
        a given platoon 
        """
        sortd = self.veh_pos_pairs(self._ID)
        topology = {}
        for item in sortd:
            current_veh = item[0]
            if current_veh == self._father_vehicle:
                topology.update({current_veh: {"front": current_veh, "leader": self._father_vehicle}})
            else:
                lane_vehicles = traci.lane.getLastStepVehicleIDs(traci.vehicle.getLaneID(current_veh))[::-1]# change to vehicles
                index = lane_vehicles.index(current_veh)-1
                preceeding_veh = lane_vehicles[index]
                topology.update({current_veh: {"front": preceeding_veh, "leader": self._father_vehicle}})
        return topology

    def pla_speed_spacing(self, topology, states):
        '''
        This funnction is designed to control platoon speed and spacing
        This method is invoked every time step to ensure platoons have the 
        most current speed and spacing commands. It can control primary and 
        secondary platoon where a given lane contains more than one platoon. 
        It also controls the platoon in the presence of obstructions by adjusting 
        the speed of the platoon to avoid collisions with the obstructing vehicle
        '''      
        vehicles = self._members
        lane = self._ID
        first_of_lane = traci.lane.getLastStepVehicleIDs(lane)[::-1][0]
        ## Check if vehicle is obstructed
        try:
            vehicles_0_states = states[vehicles[0]][-1]
        except IndexError:
            print("{} has no avoidance states entering no prior state control".format(vehicles[0]))
            if vehicles[0] == first_of_lane: ## There is no preceeding obstructing vehicle on the lane
                for vehicle in vehicles:
                    lane_num = traci.vehicle.getLaneID(vehicle).split("_")[1]
                    if traci.vehicle.getRouteID(vehicle).startswith(':'):
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                        continue
                    # Vehicles controlled here are lane leaders
                    if vehicle == vehicles[0]:
                        veh_route_edges = traci.vehicle.getRoute(vehicle)
                        veh_rou_index = traci.vehicle.getRouteIndex(vehicle)
                        next_veh_edge = veh_route_edges[veh_rou_index + 1]
                        next_veh_lane = next_veh_edge.split("_")[0] + "_" + lane_num
                        if traci.lane.getLastStepVehicleNumber(next_veh_lane) == 0: # There is no vehicle on the nexl edge lane
                            if lane_num == "0":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))
                            if lane_num == "1":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))
                            if lane_num == "2":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))        
                            if lane_num == "3":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 10)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))
                        elif traci.lane.getLastStepVehicleNumber(next_veh_lane) != 0:# and states[self._father_vehicle][-1] not in ["transit0","transit1", "transit2"]: # There is a vehicle on the next lane edge
                            last_veh_next_lane = traci.lane.getLastStepVehicleIDs(next_veh_lane)[::-1][-1]
                            if utils.get_distance(vehicle, last_veh_next_lane) > 500: # If far enough control platoon as normal
                                if lane_num == "0":
                                    #utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED) # check impact of setting speed in this case
                                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))
                                if lane_num == "1":
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))     
                                if lane_num == "2":
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))                                   
                                if lane_num == "3":
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))
                            else: # set the platoon leaders speed to speed of veh ahead
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.3) # abrupt change causes collisions change to average or headway
                                print("{} under primary speed synch control with {}".format(vehicle, last_veh_next_lane))
                    else: ## Vehicle is a follower and CACC controlled
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                topology = topology
                return topology
            ### Secondary Platoon Control
            if vehicles[0] != traci.lane.getLastStepVehicleIDs(lane)[::-1][0]: ## There is a preceeding vehicle or platoon
                index = traci.lane.getLastStepVehicleIDs(lane)[::-1].index(vehicles[0])
                last_veh_plat_ahead = traci.lane.getLastStepVehicleIDs(lane)[::-1][index -1]
                for vehicle in vehicles:
                    lane_num = traci.vehicle.getLaneID(vehicle).split("_")[1]
                    if vehicle == vehicles[0]: # 
                        # Vehicles controlled here are secondary platoons with same route as lane leader and are close enough
                        if traci.vehicle.getRouteID(vehicle) == traci.vehicle.getRouteID(last_veh_plat_ahead) \
                        and traci.vehicle.getTypeID(last_veh_plat_ahead)== "vtypeauto":
                            if utils.get_distance(vehicle, last_veh_plat_ahead) < 200:
                                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                                utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                                print("{} under Sec same route plat control gap less than 100".format(vehicle))
                            elif utils.get_distance(vehicle, last_veh_plat_ahead) > 200:
                                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under Sec same route plat control gap greater than 100".format(vehicle))
                        elif  traci.vehicle.getTypeID(last_veh_plat_ahead) != "vtypeauto": #Check for redundancy
                            utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                            utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                            print("{} under Sec diff route plat control elif".format(vehicle))
                        else:
                            # Vehicles controlled here are temporary secondary platoon leaders in same lane but diff routes as lane leader
                            utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                            utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                            print("{} under Sec diff route plat control else".format(vehicle))
                    else:
                        # All secondary followers control
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC) 
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                return topology
        else: # The platoon is not obstructed by a preceeding vehicle
            # Primary platoon control
            if vehicles[0] == first_of_lane and states[vehicles[0]][-1] not in ["transit0","transit1", "transit2"]: ## There is no preceeding vehicle on the lane
                print("{} has prior states and not in transit 0 1 or 2 in primary, state is {}".format(vehicles[0], states[vehicles[0]][-1]))
                for vehicle in vehicles:
                    lane_num = traci.vehicle.getLaneID(vehicle).split("_")[1]
                    if traci.vehicle.getRouteID(vehicle).startswith(':'):
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                        continue
                    # Vehicles controlled here are lane leaders
                    if vehicle == vehicles[0]:
                        veh_route_edges = traci.vehicle.getRoute(vehicle)
                        veh_rou_index = traci.vehicle.getRouteIndex(vehicle)
                        next_veh_edge = veh_route_edges[veh_rou_index + 1]
                        next_veh_lane = next_veh_edge.split("_")[0] + "_" + lane_num
                        if traci.lane.getLastStepVehicleNumber(next_veh_lane) == 0: # There is no vehicle on the nexl edge lane
                            if lane_num == "0":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))
                            if lane_num == "1":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))
                            if lane_num == "2":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 5)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))                        
                            if lane_num == "3":
                                utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED + 10)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under full primary control".format(vehicle))
                        elif traci.lane.getLastStepVehicleNumber(next_veh_lane) != 0:
                            last_veh_next_lane = traci.lane.getLastStepVehicleIDs(next_veh_lane)[::-1][-1]
                            if utils.get_distance(vehicle, last_veh_next_lane) > 500: 
                                if lane_num == "0":
                                    utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))
                                if lane_num == "1":
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))     
                                if lane_num == "2":
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))                                  
                                if lane_num == "3":
                                    utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.8)
                                    print("{} under primary observing control".format(vehicle))
                            else: # set the platoon leaders speed to speed of veh ahead
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.3) #
                                print("{} under primary speed synch control with {}".format(vehicle, last_veh_next_lane))
                    else: ## Vehicle is a follower and CACC controlled
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                topology = topology
                return topology
            ### Secondary Platoon Control
            if vehicles[0] != traci.lane.getLastStepVehicleIDs(lane)[::-1][0] and states[self._members[0]][-1] not in ["transit0","transit1", "transit2"]: ## There is a preceeding vehicle or platoon
                index = traci.lane.getLastStepVehicleIDs(lane)[::-1].index(vehicles[0])
                last_veh_plat_ahead = traci.lane.getLastStepVehicleIDs(lane)[::-1][index -1]
                for vehicle in vehicles:
                    lane_num = traci.vehicle.getLaneID(vehicle).split("_")[1]
                    if vehicle == vehicles[0]: # 
                        # Vehicles controlled here are secondary platoons with same route as lane leader and are close enough
                        if traci.vehicle.getRouteID(vehicle) == traci.vehicle.getRouteID(last_veh_plat_ahead) \
                        and traci.vehicle.getTypeID(last_veh_plat_ahead)== "vtypeauto":
                            if utils.get_distance(vehicle, last_veh_plat_ahead) < 200:
                                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC)
                                utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                            elif utils.get_distance(vehicle, last_veh_plat_ahead) > 200:
                                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                                print("{} under Sec same route plat control gap greater than 100".format(vehicle))
                        elif  traci.vehicle.getTypeID(last_veh_plat_ahead) != "vtypeauto": 
                            utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED) 
                            utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                            print("{} under Sec diff route plat control elif".format(vehicle))
                        else:
                            # Vehicles controlled here are temporary secondary platoon leaders in same lane but diff routes as lane leader
                            utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                            utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                            utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                            print("{} under Sec diff route plat control".format(vehicle))
                    else:
                        # All secondary followers control
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC) 
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                return topology
            # Transitory control
            if  states[self._members[0]][-1] == "obstructed" : #Vehicles on lane 3 may be exlcuded
                print("{} has prior states and in transit 0 1 or 2 in transitory, state is {}".format(vehicles[0], states[vehicles[0]][-1]))
                for vehicle in vehicles:
                    if vehicle == vehicles[0]:
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                        utils.set_par(vehicle, cc.PAR_CC_DESIRED_SPEED, self.PSPEED)
                        utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.5)
                        print("{} under transitory control and checking avoidance".format(vehicle))
                    else:
                        # All secondary followers control
                        utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.CACC) 
                        utils.set_par(vehicle, cc.PAR_CACC_SPACING, self.DISTANCE)
                topology = topology
                return topology

    def near_flag(self):
        """Determines whether a plane is on a flag bearing edge"""
        vehicles = self._members
        leader = vehicles[0]
        flag_edges = ["n3", "n8", "n13", "n18"]
        if traci.vehicle.getRoadID(leader) in flag_edges:
            return True

    def look_for_flags(self, pois, step):
        """
        Evaluates whether a plane is less than 70 meters
        from or past a flag within which lane change is permisssible
        """
        vehicles = self._members
        leader = vehicles[0]
        flag_found = False
        poi_index = "0"
        vehicle_data = utils.get_par(leader, cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data)
        time_to_pass = (((self.DISTANCE + self.VLENGTH)* self.N_VEHICLES)/v)
        for poi in pois:
            if self.states[-1] == self.FLAG_PASSED:
                continue
            if utils.get_dist_to_POI(leader, poi) < 70:
                poi_index= int(poi.split("_")[2])
                flag_found = True
                self.time_flags_found[poi_index].append(traci.simulation.getCurrentTime()/1000)
                if self.time_flags_found[poi_index][-1] - self.time_flags_found[poi_index][-2] < time_to_pass:
                    self.states.append(self.FLAG_PASSED)
                    poi_index= int(poi.split("_")[2])
                    flag_found = False
        flag_n_poi_index = [flag_found, poi_index]
        return flag_n_poi_index

    def safe_to_cl(self):
        """
        Checks the occupancy of the desired new lane If empty returns True,
        returns false if one or more vehicles are found
        """
        vehicles = self._members
        leader = vehicles[0]
        leader_pos = traci.vehicle.getLanePosition(leader)
        lane = self._ID
        lane_index = lane.split("_")[1]
        next_best_lane = lane.split("_")[0]+ "_" + str(int(lane_index)-1)
        vehicles_on_next_lane = traci.lane.getLastStepVehicleIDs(next_best_lane)
        if vehicles_on_next_lane != []:           
            for vehicle in vehicles_on_next_lane:
                if traci.vehicle.getLanePosition(vehicle) <= leader_pos and traci.vehicle.getLanePosition(vehicle) > leader_pos - (self.DISTANCE+self.VLENGTH)*self.N_VEHICLES:
                    print("Obtructing Vehs are {}".format(vehicles_on_next_lane))
                    return False               
            return True
        else:        
            return True 

    def actual_poi(self):
        """Determine which poi platoon is approaching"""
        vehicles = self._members
        leader = vehicles[0]
        curr_edge = traci.vehicle.getRoadID(leader)
        poi_edges = [('exit_POI_0', 'n3'), ('exit_POI_1', 'n8'), ('exit_POI_2', 'n13'), ('exit_POI_3', 'n18')]
        actual_poi = [item[0] for item in poi_edges if item[1] == curr_edge]
        return actual_poi

    def plane_obstructed(self, states, obstructors):
        """
        Checks if there is a nonauto veh ahead and within a distance to the leader on
        the current lane if the lane allows obtructed veh avoidance. Returns true if there 
        is or if the previous state of the leader was obstructed or transitory
        states: defaultdict list [leader, [states]] which can either be obstructed, transit0, transit1, transit2 or completed
        obstructors: defaultdict list [leader, [obstructing veh ID]]
        """
        leader = self._members[0]
        lane = self._ID
        try:
            leader_curr_state = states[leader][-1]
            print("plane_obs try leader state is {}".format(leader_curr_state))
        except IndexError: # First timers
            lane_index = lane.split("_")[1]

            lane_vehicles = traci.lane.getLastStepVehicleIDs(lane)
            leader_lane_index = lane_vehicles.index(leader)
            try:
                obst_veh = lane_vehicles[leader_lane_index + 1]
            except IndexError:
                print("No obstructing vehicle infront of {}".format(leader))
                return False
            else: # repeat visitor Veh isnt from a platoon and platoon members are all on lane and dist to end of lane is more than 500
                print("{} is obstructed, checking lane position".format(leader))
                if traci.vehicle.getTypeID(obst_veh) != "vtypeauto":# and traci.vehicle.getLanePosition(leader) > (self.DISTANCE+self.VLENGTH)*self.N_VEHICLES and traci.lane.getLength(lane) - traci.vehicle.getLanePosition(leader) > 500:
                    print("{} is obstructed, no states".format(leader))
                    states[leader].append("obstructed")
                    obstructors[leader].append(obst_veh)
                    traci.vehicle.setLaneChangeMode(obst_veh, COOP_LC)
                    return True
                else:
                    return False
        else:# 
            print("{} checking if in obstructed, transit 0, 1 or 2 state is {}".format(leader, states[leader][-1]))
            if states[leader][-1] == "obstructed":
                # Add constraints here to detected failed avoidances, delete their states and return them to normal speed control
                lane_vehicles = traci.lane.getLastStepVehicleIDs(lane)
                leader_lane_index = lane_vehicles.index(leader)
                try:
                    obst_veh = lane_vehicles[leader_lane_index + 1]
                except:
                    print("Obs no longer present")
                else:
                    leader_lane_pos= traci.vehicle.getLanePosition(leader)
                    obs_veh_lane_pos = traci.vehicle.getLanePosition(obst_veh)
                    if leader_lane_pos > obs_veh_lane_pos:
                        traci.vehicle.setLaneChangeMode(obst_veh, COOP_LC)
                        print("{} is obstructed, recurrent {}".format(leader, states[leader][-1]))
                        return True
            else:
                del states[leader]
                return False

    def obst_overtaken(self, obstructors, states):
        """Evaluates whether a platoon have overtaken the obstructing vehicle"""
        vehicles = self._members
        leader = self._members[0]
        lane = self._ID
        obs_veh = obstructors[leader][-1]
        leader_lane_pos= traci.vehicle.getLanePosition(leader)
        obs_veh_lane_pos = traci.vehicle.getLanePosition(obs_veh)
        if leader_lane_pos > obs_veh_lane_pos and utils.get_distance(leader, obs_veh) > (self.DISTANCE + self.VLENGTH)* (len(vehicles) + 4): # 4 safety gap
            states[leader].append("overtaken")
            del obstructors[leader]
            return True
        else:
            return False

    def move_to_next_best_lane(self, step):
        """Change travel lane of platoon toone lane down"""
        vehicle_data = utils.get_par(self._members[0], cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data) 
        time_to_pass = (((self.DISTANCE + self.VLENGTH)* self.N_VEHICLES)/v)
        vehicles = self._members
        pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
        current_lane_num = traci.vehicle.getLaneIndex(vehicles[0])
        next_best_lane = current_lane_num - 1
        for vehicle in vehicles:
            utils.change_lane(vehicle, next_best_lane)

    def set_arrived_free(self):
        """Set arriving vehicle active controllers to ACC"""
        vehicles = self._members
        leader = vehicles[0]
        if traci.vehicle.getLaneID(leader).split("_")[1] == "0":
            for vehicle in vehicles:
                utils.set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
                utils.set_par(vehicle, cc.PAR_ACC_HEADWAY_TIME, 0.2)
                #utils.set_par(vehicle, cc.PAR_FIXED_ACCELERATION, 1)
            print("Vehicles set free : {}".format(vehicles))

def remove_obstructors():
    """Removes Obstructer vehicles from network after their task is complete"""
    flag_edges = ["n3", "n8", "n13", "n18"]
    for edge in flag_edges:
        vehs_in_flag_edges = traci.edge.getLastStepVehicleIDs(edge)
        non_autos = [vehicle for vehicle in vehs_in_flag_edges if traci.vehicle.getTypeID(vehicle) != "vtypeauto"]
        for vehicle in non_autos:
            if traci.vehicle.getLanePosition(vehicle) >= 100:
                traci.vehicle.remove(vehicle, REMOVE_PARKING)
