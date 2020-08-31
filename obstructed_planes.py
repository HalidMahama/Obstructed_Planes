import pdb
import os
import sys
import ccparams as cc
import random
import time
import planers
from collections import defaultdict

from utils import add_vehicle, set_par, change_lane, communicate, \
    get_distance, get_par, start_sumo, running, validate_params, retrieve_vehicles, \
    filter_cacc_vehicles, get_dist_to_POI_old, get_dist_to_POI_new

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib
import traci

# Length of vehicles
LENGTH = 4
# inter-vehicluar gap
DISTANCE = 5
# cruise speed
SPEED = 35
PSPEED = 48
ALL_CHECKS_SPEED_MODE = 1111 
REMOVE_PARKING = 0x01

FIX_LC = 0b0000000000

# PLANE STATES
# If the plane state is 0: there are no active planes to control
# If the plane state is 1: platoons are formed and lane changes are forbiden
# If the plane state is 2: vehicles are acc controlled and are able to change lanes
NON = 0
PLATOONING = 1
NONPLATOONING = 2

# VEHICLE STATES
# If the vehicle state is 0: It is platooning and only controlled by plane logic
# If the vehicle state is 1: It is in a nonplatooning lane, acc controlled and can change lanes
# If the vehicle state is 2: It is a free agent, Driver controlled and obeys no plane logic
IDLE = 0
MANEUVERING = 1
FREE_AGENT = 2

N_VEHICLES = 24
N_VEHICLES_GEN = 24
SOURCES = ["p0", "s0"]
pois = ["exit_POI_0", "exit_POI_1", "exit_POI_2", "exit_POI_3"]
#flags_at_pois = {}
time_flags_found =[0]

PLAT_EDGES =  ["p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19"]
ARR_EDGES = ["e0", "exit0", "e1", "exit1", "e2", "exit2", "e3", "exit3", "n3", "n8", "n13","n18"]
ADD_PLAT_STEP = 700

# sumo launch command
sumoBinary = sumolib.checkBinary('sumo-gui')
sumoCmd = [sumoBinary, "D", "-c", "cfg/freeway.sumo.cfg"]
states = defaultdict(list) # This distionary is only to be used for avoidance purposes, it should remain empty if vehicle isn't trying to avoid obs veh
obstructors = defaultdict(list)
avoidance = defaultdict(list)
vois = defaultdict(list)

def add_vehicles(n, batch_num, platoon_len, fromEdge, real_engine):
    # route param: for each source edge there are four possible routes
    start_from = n*batch_num
    end_at = start_from + platoon_len
    
    index = fromEdge.split("e")[1]
    if index == "0":
        exitEdges =['exit0', 'exit1', 'exit2']
    elif index=="1":
        exitEdges =['exit1', 'exit2', 'exit3']
    elif index == "2":
        exitEdges =['exit2', 'exit3', 'exit1']
    else: # index = "3"
        exitEdges =['exit3', 'exit1', 'exit2']

    for i in range(start_from, end_at):
        lane = 1
        vid = "v.%d" % i
        toEdge= exitEdges[0]
        route = "route_"+index+"_"+str(lane-1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

    start_from = start_from + platoon_len
    end_at = start_from + platoon_len

    for i in range(start_from, end_at):
        lane = 2
        vid = "v.%d" % i
        toEdge= exitEdges[1]
        route = "route_"+index+"_"+str(lane-1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

    start_from = start_from + platoon_len
    end_at = start_from + platoon_len

    for i in range(start_from, end_at):
        lane = 3
        vid = "v.%d" % i
        toEdge= exitEdges[2]
        route = route = "route_"+index+"_"+str(lane-1)
        add_vehicle(vid, route, (end_at - i + 1) * (DISTANCE + LENGTH) +
                    100, lane, SPEED, DISTANCE, real_engine)
        set_par(vid, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)
        change_lane(vid, lane)

def lane_gen(lanes_per_edge=4): 

    edges = ["source0", "s0", "source1", "s1", "source2", "s2" , "source3", "s3", "p0", "n1", "p2", "n3", "p4", "p5", "n6", "p7", "n8", "p9", "p10", "n11", "p12", "n13", "p14", "p15", "n16", "p17", "n18", "p19" ]
    lanes =[]
    for edge in edges:
        for lane in range(1,lanes_per_edge):
            laneID = edge + "_" + str(lane)
            lanes.append(laneID)
    return lanes

def set_lc_mode():
    obst_edges = ["non0", "non1", "non2", "non3"]
    for edge in obst_edges:
        edge_vehs = traci.edge.getLastStepVehicleIDs(edge)
        if edge_vehs != []:
            for vehicle in edge_vehs:
                traci.vehicle.setLaneChangeMode(vehicle, FIX_LC)
        else:
            continue

def proute_assigner(SOURCES):
    routes = ["route0", "route1", "route2", "route3"]
    for edge in SOURCES:
        vehicles = traci.edge.getLastStepVehicleIDs(edge)
        #print("The vehs at source are : {}".format(vehicles))
        for vehicle in vehicles:
            if traci.vehicle.getTypeID(vehicle) != "vtypeauto":
                lane_vehicles.pop(lane_vehicles.index(vehicle))
        for vehicle in vehicles:
            route = routes[int(traci.vehicle.getLaneID(vehicle).split("_")[1]) - 1]
            traci.vehicle.setRouteID(vehicle, route)
            #print(" The best lanes for {} is {} ".format(vehicle, route))

def sorted_planes(lane_vehicles, lane, removed_vehicles):
    planes=[]
    #routes =["route0", "route1", "route2", "route3"]
    vehicles = lane_vehicles
    primary_plane = []
    secondary_plane = []
    vehicles = [vehicle for vehicle in vehicles if vehicle not in removed_vehicles]
    vehicles = [vehicle for vehicle in vehicles if traci.vehicle.getTypeID(vehicle) == "vtypeauto"]# and traci.vehicle.getRouteID(vehicle) == leader_route]
    if vehicles != []:
        print("vehs[0] is {}".format(vehicles[0]))
        print("vehs[0] rout is {}".format(traci.vehicle.getRouteID(vehicles[0])))
        leader_route = traci.vehicle.getRouteID(vehicles[0])
        for vehicle in vehicles:
            veh_lane_index = vehicles.index(vehicle)
            if traci.vehicle.getRouteID(vehicle) == leader_route and veh_lane_index < N_VEHICLES: #get_distance(vehicle, vehicles[0]) < (LENGTH*N_VEHICLES +(DISTANCE*(N_VEHICLES-1)) + 50): # 200 + length of plat
                primary_plane.append(vehicle)
            else: 
                secondary_plane.append(vehicle)

        ps_planes = [primary_plane, secondary_plane]
        all_planes = []

        for item in ps_planes:
            #print("ps item {}".format(item))
            item_planes_with_empties = [(item[N_VEHICLES_GEN*i: N_VEHICLES_GEN*i + N_VEHICLES_GEN]) for i in range(N_VEHICLES_GEN)]
            item_planes = [plane for plane in item_planes_with_empties if plane != []]
            #print("item_plane:{}".format(item_planes))

            for plane in item_planes:
                #print("plane:{}".format(plane))
                planes.append(planers.Plane(
                                lane, plane , lane_spacing=4, lane_speed=SPEED, platoonable=False))
        return planes

    else:
        planes = []
        return planes

def look_for_flags(leader, pois, flags_at_pois, step):
    flag_edges = ["n3", "n8", "n13", "n18"]
    flag = False
    vehicle = leader

    if vehicle in flags_at_pois.keys() and flags_at_pois[vehicle]['state'][-1] == 'completed':# and traci.simulation.getCurrentTime()/1000 - flags_at_pois[leader]['time'][-1] < time_to_pass : # Might lead to false false flags if speed is low and time to pass increases
        vehicle_data = get_par(vehicle, cc.PAR_SPEED_AND_ACCELERATION)
        (v, a, u, x1, y1, t) = cc.unpack(vehicle_data) 
        time_to_pass = (((DISTANCE + LENGTH)* N_VEHICLES)/v)
        print("time to pass is {} ".format(time_to_pass))
        print("veh {} states are {}".format(vehicle, flags_at_pois[vehicle]['state']))
        print("veh {} poitypes are {}".format(vehicle, flags_at_pois[vehicle]['poitype']))
        print("time to pass is {} ".format(time_to_pass))
        print("time since last change is {}".format(traci.simulation.getCurrentTime()/1000 - flags_at_pois[leader]['time'][-1]))
        #if traci.simulation.getCurrentTime()/1000 - flags_at_pois[leader]['time'][-1] < time_to_pass :
        curr_edge = traci.vehicle.getRoadID(vehicle)
        poi_edges = [('exit_POI_0', 'n3'), ('exit_POI_1', 'n8'), ('exit_POI_2', 'n13'), ('exit_POI_3', 'n18')]
        
        actual_poi = [item[0] for item in poi_edges if item[1] == curr_edge]
        print("actual poi is {}".format(actual_poi))
        print("in flag poi is {}".format(flags_at_pois[vehicle]['poi'][-1]))
        if actual_poi[-1] == flags_at_pois[vehicle]['poi'][-1]:

            print("veh {} has false flag identified".format(vehicle))
            flag = False
            flag_data = (flag, flags_at_pois[vehicle]['poitype'][-1])
            return flag_data

        else: # This entry isn't valid actual flag and dictionary entry mismatch
            del flags_at_pois[vehicle]
            flag = False
            flag_data = (flag, 'dummy')

    else:
        d = defaultdict(list)
        if traci.vehicle.getRoadID(vehicle) in flag_edges:

            for poi in pois:
                if get_dist_to_POI_new(vehicle,poi) <= 150:
                    print("Distance to poi new is{}".format(get_dist_to_POI_new(vehicle,poi)))
                    #print("Distance to poi old is{}".format(get_dist_to_POI_old(vehicle,poi)))

                    ctime = traci.simulation.getCurrentTime()/1000
                    poitype = traci.poi.getType(poi)
                    s = [('poi', poi), ('poitype', poitype),('time', ctime), ('state', 'InZone')]
                    for k, v in s:
                        d[k].append(v)

                    flags_at_pois[vehicle] = dict(d.items())
                    poi_pos = traci.poi.getPosition(poi)
                    veh_pos = traci.vehicle.getPosition(vehicle)
                    if flags_at_pois[vehicle]['poitype'][-1] == 'WE' and traci.vehicle.getRoadID(vehicle) == 'n3':
                        if flags_at_pois[vehicle]['state'][-1] == 'InZone':
                            print("Yaaaaaaaaay")
                            flags_at_pois[vehicle]['state'][-1] = 'changed'
                            print("Flags at Yaaaaaaaaay is {}".format(flags_at_pois))
                            flag = True
                            flag_data = (flag, poitype)
                            return flag_data
                    if flags_at_pois[vehicle]['poitype'][-1] == 'NS' and traci.vehicle.getRoadID(vehicle) == 'n8':
                        if flags_at_pois[vehicle]['state'][-1] == 'InZone':
                            print("Yaaaaaaaaay")
                            flags_at_pois[vehicle]['state'][-1] = 'changed'
                            print("Flags at Yaaaaaaaaay is {}".format(flags_at_pois))
                            flag = True

                            flag_data = (flag, poitype)
                            return flag_data
                    if flags_at_pois[vehicle]['poitype'][-1] == 'EW' and traci.vehicle.getRoadID(vehicle) == 'n13':
                        if flags_at_pois[vehicle]['state'][-1] == 'InZone':
                            print("Yaaaaaaaaay")
                            flags_at_pois[vehicle]['state'][-1] = 'changed'
                            print("Flags at Yaaaaaaaaay is {}".format(flags_at_pois))
                            flag = True
                            flag_data = (flag, poitype)
                            return flag_data
                    if flags_at_pois[vehicle]['poitype'][-1] == 'SN' and traci.vehicle.getRoadID(vehicle) == 'n18':
                        if flags_at_pois[vehicle]['state'][-1] == 'InZone':
                            print("Yaaaaaaaaay")
                            flags_at_pois[vehicle]['state'][-1] = 'changed'
                            print("Flags at Yaaaaaaaaay is {}".format(flags_at_pois))
                            flag = True
                            flag_data = (flag, poitype)
                            return flag_data


def set_arrived_free(ARR_EDGES):
    for edge in ARR_EDGES:
        lane_of_interest = edge +"_0"
        vehicles = traci.lane.getLastStepVehicleIDs(lane_of_interest)
        for vehicle in vehicles:
            if traci.vehicle.getTypeID(vehicle)=="vtypeauto":
                set_par(vehicle, cc.PAR_ACTIVE_CONTROLLER, cc.ACC)


def main(real_engine, setter=None, demo_mode= False):

    start_sumo("cfg/freeway.sumo.cfg", False)
    step = 0
    batch_num = 0
    veh_of_interest = "v.40"
    source_edges = ['source0', 'source1', 'source2', 'source3']
    edge_filter, vtype_filter = validate_params(
        edge_filter=PLAT_EDGES, vtype_filter=["vtypeauto"])
    pstate = NON

    while running(demo_mode, step, 4132): 

        if demo_mode and step == 4132: 
            start_sumo("cfg/freeway.sumo.cfg", False)
            step = 0       
            flags_at_pois = {}
        if pstate == NON:
            lanes = lane_gen()
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[0], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3 ### Start from here
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[1], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[2], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3
            add_vehicles(N_VEHICLES_GEN, batch_num, fromEdge = source_edges[3], platoon_len = 24, real_engine = False)
            batch_num = batch_num + 3
            traci.gui.setZoom("View #0", 4500)
            # f = open("/Users/mac/src/Simulations/mixedtraffic/gen_times","w")
            # f.write("simulation step is {}".format(traci.simulation.getCurrentTime()))
            # f.close()
            #traci.gui.setZoom("View #1", 4500)
            topology = {}
            flags_at_pois = {}
            teleported_vehicles =[]
            vstate = IDLE
            pstate = PLATOONING
            genStep = step
            print("Gen Step is : {}".format(genStep))
            print("pstate at gen is : {}".format(pstate))
        if pstate == PLATOONING and step == genStep + 1:
            veh_of_interest = traci.lane.getLastStepVehicleIDs('source0_3')[::-1][0]
            print("veh of interest is: {}".format(veh_of_interest))

        if pstate == PLATOONING:
            traci.simulationStep()

        if step > genStep + 10 and pstate == PLATOONING:
            print("veh of int is:{}".format(veh_of_interest))
            if veh_of_interest in traci.edge.getLastStepVehicleIDs("p12"):
                print("veh of interest at set location {}!!!!!!!!!!!".format(traci.vehicle.getLaneID(veh_of_interest)))
                pstate = NON

            if step <= genStep + 14:
                set_lc_mode()
                print("LC Mode Set to FIX_LC")
            list_of_leaders =[]
            for lane in lanes:
                if traci.lane.getLastStepVehicleIDs(lane)==[]:
                    continue
                lane_vehicles = traci.lane.getLastStepVehicleIDs(lane)[::-1]
                teleported_vehicles = traci.simulation.getEndingTeleportIDList()
                print("end teleported {}".format(teleported_vehicles))
                teleported_vehicles = [vehicle for vehicle in teleported_vehicles if vehicle not in removed_vehicles]
                print("Teleported {}".format(teleported_vehicles))
                removed_vehicles = []
                for vehicle in teleported_vehicles:
                    try:
                        traci.vehicle.remove(vehicle, REMOVE_PARKING)
                    except:
                        print("vehicle already removed")
                    else:
                        removed_vehicles.append(vehicle)
                        print("vehicle {} has been removed".format(vehicle))
                #teleported_vehicles = [vehicle for vehicle in end_teleport_vehicles if vehicle in teleported_vehicles and vehicle not in removed_vehicles]
                if lane_vehicles != []:
                    planes = sorted_planes(lane_vehicles, lane, removed_vehicles)
                if planes != []:                  
                    for plane in planes:
                        topology = plane.topo_contsructor()
                        topology = plane.pla_speed_spacing(topology, states)
                        #print("Topology at platSpeedSpacing is : {}".format(topology))
                        communicate(topology)
                        set_arrived_free(ARR_EDGES) 
                    for plane in planes:                      
                        #print("states are {}".format(states))
                        if traci.vehicle.getRouteID(plane.plane_leader()).split("_") == '0':
                            continue 
                        if plane.near_flag():
                            leader = plane.plane_leader()
                            print("Veh {} looking for flag".format(leader))
                            flag_data = look_for_flags(leader,pois,flags_at_pois, step)
                            if flag_data != None:
                                if flag_data[0] == True and plane.safe_to_cl():
                                    print("Veh {} has found a flag, changing lanes now".format(leader))
                                    plane.move_to_next_best_lane(step)
                                    flags_at_pois[leader]['state'].append('completed')
                                    flags_at_pois[leader]['poitype'].append(flag_data[1])
                                elif flag_data[0] == True and plane.safe_to_cl() == False:
                                    print(plane.safe_to_cl())
                        #pdb.set_trace()           
                        obstruction = plane.plane_obstructed(states, obstructors)
                        if obstruction == True:
                            plane.obst_overtaken(obstructors, states)
                        planers.remove_obstructors()
                        #pdb.set_trace()
                    traci.simulationStep()              
        print("step is : {}".format(step))
        print("Current time is :{}".format(traci.simulation.getCurrentTime()))
        print("pstate is : {}".format(pstate))
        step += 1       
    traci.close()

if __name__ == "__main__":
    main(True, True)
