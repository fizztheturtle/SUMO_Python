from __future__ import absolute_import
from __future__ import print_function
import xml.etree
import io
import os
import sys
import optparse
import traci
import csv

output_filename = 'Eval_500_EV_CS_thresh_0.5'
threshold = 0.5

THISDIR = os.path.dirname(__file__)
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))
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

charging_stations = []

# extract all charging station edge and position from additionals file
# and put each position and edge of charging station into a list.
for root, dirs, files in os.walk('FiveByFiveSquares'):
    for file in files:
        if file.endswith('additional_cs_veh.xml'):
            F = open('FiveByFiveSquares/' + file, 'r')

            with io.open('FiveByFiveSquares/' + file, 'r', encoding='utf-8-sig') as f:
                contents = f.read()
                tree = xml.etree.ElementTree.fromstring(contents)

                for level in tree.findall('./chargingStation'):
                    cs_id = level.get('id')
                    cs_lane = level.get('lane')
                    cs_start_pos = level.get('startPos')
                    cs_end_pos = level.get('endPos')
                    charging_stations.append([cs_id, cs_lane, cs_start_pos, cs_end_pos])


def last_occurrence(veh_id_arr, veh_id):
    last_index = len(veh_id_arr) - veh_id_arr[::-1].index(veh_id) - 1

    return last_index


# remove an electric vehicle if the vehicle has 0 charge left
def remove_ev(veh_id):
    traci.vehicle.remove(veh_id, 3)


# Will retrieve the current metres per second of the chosen vehicle
def get_speed_mps(veh_id):
    get_current_speed_mps = float(traci.vehicle.getSpeed(veh_id))

    return get_current_speed_mps


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


def ev_charging_station_stop(veh_id, veh_last_edge):
    travel_distance_array = []
    veh_lane_index = traci.vehicle.getLaneID(veh_id)
    veh_lane_position = traci.vehicle.getLanePosition(veh_id)
    changed = False

    for cs in charging_stations:
        travel_distance = traci.simulation.getDistanceRoad(veh_lane_index[:-2], veh_lane_position, cs[1][:-2],
                                                           float(cs[2]), True)
        travel_distance_array.append([int(travel_distance), veh_id, cs[0], cs[1], float(cs[2])])

    travel_distance_array.sort()

    for min_travel_distance in travel_distance_array:

            if min_travel_distance[0] == 1.7976931348623157e+308 and veh_lane_index == min_travel_distance[3]\
                    and 1 <= veh_lane_position <= 85:
                print(veh_lane_position)
                print(veh_id, veh_lane_index, min_travel_distance[3], ', stop at cs2')

                # 18000000 milliseconds= 30 min stop
                traci.vehicle.setChargingStationStop(veh_id, min_travel_distance[2], duration=1800000)
                traci.vehicle.changeTarget(veh_id, veh_last_edge)
                changed = True

                break

            elif 1 <= min_travel_distance[0] <= 85and veh_lane_index == min_travel_distance[3]:

                print(veh_id, veh_lane_index, min_travel_distance[0], min_travel_distance[2], ', stop at cs3')
                traci.vehicle.setChargingStationStop(veh_id, min_travel_distance[2], duration=1800000)
                traci.vehicle.changeTarget(veh_id, veh_last_edge)
                changed = True
                break

    if changed is False:

        traci.vehicle.changeTarget(veh_id, travel_distance_array[0][3][:-2])


def run():
    veh_id_arr = []
    curr_bat_capac_arr = []

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
            range_calc_cons = range_calc

            if electricity_consumption > 0 and get_current_speed_mps > 1:

                veh_edge = traci.vehicle.getLaneID(veh_id)[:-2]

                veh_lane_position = traci.vehicle.getLanePosition(veh_id)

                remaining_distance = traci.simulation.getDistanceRoad(veh_edge, int(veh_lane_position),
                                                                      veh_last_edge, 83, isDriving=True)

                # The following lines of code, are to find the first and last value of each attribute of the vehicle
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

                # If there is no charge remaining or the vehicle has reached its destination, remove from the simulation
                if current_battery_capacity == 0:

                    id_list.remove(veh_id)
                    remove_ev(veh_id)
                    break

                # set a threshold here for testing
                elif (range_calc_cons - (range_calc_cons * threshold)) < remaining_distance:
                    # continuously check whether  the battery of the current vehicle can reach its desired destination
                    ev_charging_station_stop(veh_id, veh_last_edge)

                # set a threshold here for testing
                elif (range_calc_cons - (range_calc_cons * threshold)) >= remaining_distance:

                    if traci.vehicle.getRoute(veh_id)[-1] == veh_last_edge:
                        continue
                    else:
                        traci.vehicle.changeTarget(veh_id, veh_last_edge)

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
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
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
