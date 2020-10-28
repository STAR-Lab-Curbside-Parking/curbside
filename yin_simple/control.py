'''
control.py
-----------------------------------------------------------------------------
Defines parking space and vehicle functionality for curb simulation via TraCI
'''

import os, sys
import argparse

# set up
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
import traci.constants as tc

import curb

def main():
    """
    Main function that controls the simulation
    """

    # list for smart curb
    smart_curb_list = ['parkingArea_1']

    # list for non-smart curb
    # stupid_curb_list = ['parkingArea_0', 'parkingArea_2']

    # subscribe to source junction of parking area
    # traci.parkingarea.subscribeContext('parkingArea_1', tc.CMD_GET_VEHICLE_VARIABLE, 100, [tc.VAR_TYPE])

    traci.junction.subscribeContext('gneJ5', tc.CMD_GET_VEHICLE_VARIABLE, 100, 
                                [tc.VAR_TYPE, tc.VAR_ROUTE_ID, tc.VAR_LENGTH, 
                                tc.VAR_POSITION, tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION])

    road_network = curb.create_graph('simple.net.xml')
    a = curb.curb('simple.add.xml', 'simple.net.xml', '1', 'truck')
    b = curb.smart_curb(1, 'simple.add.xml', 'simple.net.xml', '2', 'truck', road_network)

    print(b._reroute_choice())
    traci.close()
    exit()

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # if curb is still operating
        # via current time
        if traci.simulation.getTime():
            operating = True

        # curb based - look at the associated edge
        if traci.junction.getContextSubscriptionResults('gneJ5'):
            # sort the retrieved vehicles by distance to curb
            print(traci.junction.getContextSubscriptionResults('gneJ5'))

        # look at "truck", in real case, should loop through the subsribed dictionary
        # decision for a vehicle
        confirmed = False
        trigger = False

        # if the vehicle has not confirmed parking, plans to park at the smart curb
        if (not confirmed) and traci.vehicle.getNextStops('truck') and \
           "parkingArea_1" in [item[2] for item in traci.vehicle.getNextStops('truck')]:

            # if vehicle type matches, the curb is open and the curb is not fully occupied
            if traci.vehicle.getVehicleClass('truck') == 'passenger' and \
               operating and \
               (int(traci.simulation.getParameter('parkingArea_1', "parkingArea.occupancy")) < 1):
                    confirmed = True
            else:
                # otherwise, the parking is rejected and need to trigger a reroute
                trigger = True

        if trigger:
            # dynamically reroute to the another lot
            traci.vehicle.rerouteParkingArea('truck', 'parkingArea_2')

        # in real decision, the curb should see a list of candidate vehicles
        # retrieve a list and then take that and other information (congestion on its edge) as state to make decision

        # print(traci.simulation.getParameter('ParkAreaA', "parkingArea.lane"))
        # print(traci.simulation.getParameter('ParkAreaA', "parkingArea.occupancy"))

    # enter False to disconnect before SUMO finishes
    traci.close()

    # flush out intermmediate results
    sys.stdout.flush()

if __name__ == '__main__':

    """
    User starts the simulation with sumo or sumo-gui input
    """

    parser = argparse.ArgumentParser()
    parser.add_argument('UI', action='store', type=str, 
                        help='Option for simulation. Either sumo or sumo-gui')
    args = parser.parse_args()

    traci.start([args.UI, "-c", "simple.sumocfg"])
    main()
