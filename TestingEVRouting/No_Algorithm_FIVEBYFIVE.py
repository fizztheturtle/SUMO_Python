from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import csv
import traci

output_filename = 'Eval_500_EV_baseline'

THISDIR = os.path.dirname(__file__)
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary

except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of"
        " your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


def last_occurrence(veh_id_arr, veh_id):
    last_index = len(veh_id_arr) - veh_id_arr[::-1].index(veh_id) - 1

    return last_index


def remove_ev(veh_id):
    # remove an electric vehicle if the vehicle has 0 charge left
    traci.vehicle.remove(veh_id, 3)


# calculates the range using the equation
# Range(m)= remaining Battery capacity(Kwh)/electricity consumed(wh/second)*speed(m/second)
# electricity consumed(wh/s) is per second due to it being retrieved from the last time step.
def range_calculation(veh_id):
    range_calc = 0
    #  calculate how far an EV can travel with current capacity;
    #  battery amount divided by electricity Consumption multiplied by current speed, gives range,
    remaining_battery_capacity_kwh = float(traci.vehicle.getParameter(veh_id, 'device.battery.actualBatteryCapacity'))

    electricity_consumption_wh = float(traci.vehicle.getElectricityConsumption(veh_id))

    speed_mps = get_speed_mps(veh_id)
    if electricity_consumption_wh > 0 and speed_mps > 1:
        range_calc = (remaining_battery_capacity_kwh / electricity_consumption_wh) * speed_mps

    return range_calc


# Will retrieve the current metres per second of the chosen vehicle
def get_speed_mps(veh_id):
    get_current_speed_mps = float(traci.vehicle.getSpeed(veh_id))

    return get_current_speed_mps


def run():

    veh_id_arr = []
    curr_bat_capac_arr = []
    veh_edg_arr = []
    veh_last_edge_arr = []
    range_arr = []
    remaining_distance_arr = []

    rou_data = []

    vehicles = list(map(str, range(100 + 1)))
    get_route_edges = []

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        id_list = traci.vehicle.getIDList()

        for veh_id in id_list:
            get_route_id = traci.vehicle.getRoute(veh_id)

            for i in vehicles:
                if i == veh_id:
                    rou_data.append([veh_id, get_route_id])
                    get_route_edges.append(traci.vehicle.getRoute(veh_id))
                    vehicles.remove(i)

            try:
                veh_last_edge = get_route_edges[int(veh_id)][-1]

            except IndexError:
                veh_last_edge = traci.vehicle.getRoute(veh_id)[-1]

            get_current_speed_mps = float(traci.vehicle.getSpeed(veh_id))

            electricity_consumption = float(traci.vehicle.getElectricityConsumption(veh_id))

            current_battery_capacity = float(traci.vehicle.getParameter(veh_id, 'device.battery.actualBatteryCapacity'))

            range_calc = range_calculation(veh_id)

            if electricity_consumption > 0 and get_current_speed_mps > 1:

                veh_edge = traci.vehicle.getLaneID(veh_id)[:-2]
                veh_lane_position = traci.vehicle.getLanePosition(veh_id)
                remaining_distance = traci.simulation.getDistanceRoad(veh_edge, int(veh_lane_position),
                                                                      veh_last_edge, 83, isDriving=True)

                # The following lines of code, are to find the first and last attributes of each vehicle
                if veh_id not in veh_id_arr:
                    veh_id_arr.append(veh_id)
                    curr_bat_capac_arr.append(current_battery_capacity)

                    veh_last_edge_arr.append(traci.vehicle.getRoute(veh_id)[-1])
                    range_arr.append(range_calc)
                    remaining_distance_arr.append(remaining_distance)

                elif veh_id_arr.count(veh_id) < 2:
                    veh_id_arr.append(veh_id)
                    curr_bat_capac_arr.append(current_battery_capacity)

                    veh_last_edge_arr.append(traci.vehicle.getRoute(veh_id)[-1])
                    range_arr.append(range_calc)
                    remaining_distance_arr.append(remaining_distance)
                else:
                    last_ind = last_occurrence(veh_id_arr, veh_id)
                    del veh_id_arr[last_ind]
                    del curr_bat_capac_arr[last_ind]

                    del range_arr[last_ind]
                    del remaining_distance_arr[last_ind]

                    veh_id_arr.append(veh_id)
                    curr_bat_capac_arr.append(current_battery_capacity)

                    veh_last_edge_arr.append(traci.vehicle.getRoute(veh_id)[-1])
                    range_arr.append(range_calc)
                    remaining_distance_arr.append(remaining_distance)

                # continuously check whether the battery of the current vehicle can reach its desired destination
                if current_battery_capacity == 0 or remaining_distance == 1.7976931348623157e+308:

                    id_list.remove(veh_id)
                    remove_ev(veh_id)
                    break


    csv_out = open(output_filename+'.csv', 'w', newline='')

    # create the csv writer object.
    cvs_write = csv.writer(csv_out)

    for row in zip(veh_id_arr, curr_bat_capac_arr, veh_last_edge_arr, range_arr,remaining_distance_arr):
        cvs_write.writerow(row)

    csv_out.close()


    traci.close()
    sys.stdout.flush()


def get_options():
    """define options for this script and interpret the command line"""
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    # load whether to run with or without GUI
    options = get_options()
    # If this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    sumoCmd = [sumoBinary, "-c", "FiveByFiveSquares/evRoutingFiveByFiveKm.sumo.cfg"]

    traci.start(sumoCmd)

    run()

