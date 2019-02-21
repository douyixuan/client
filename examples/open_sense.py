__version__ = "1.0"


import json
import logging
import sys
import time

try:
    import prctl
except:
    pass

from monodrive import SimulatorConfiguration, VehicleConfiguration, Simulator
from monodrive.ui import GUI
from monodrive.util import InterruptHelper
from monodrive.vehicles import OpenSenseVehicle
from monodrive.networking.messaging import ConfigureTrajectoryCommand, StepSimulationCommand

from monodrive.networking.client import Client

ManualDriveMode = True


if __name__ == "__main__":

    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')

    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_config = VehicleConfiguration('state.json')

    client = Client((simulator_config.configuration["server_ip"],
                     simulator_config.configuration["server_port"]))

    if not client.isconnected():
        client.connect()
    simulator = Simulator(client, simulator_config)
    simulator.setup_logger()
    simulator.send_configuration()
    time.sleep(2)

    trajectory = json.load(open("Trajectory.json", "r"))
    print(client.request(ConfigureTrajectoryCommand(trajectory)))
    for i in range(0, 10):
        print(client.request(StepSimulationCommand(1)))
        time.sleep(2)

    # episodes = 1  # TODO... this should come from the scenario config
    # # Setup Ego Vehicle
    # from monodrive import VehicleConfiguration
    #
    # # Setup Ego Vehicle
    # ego_vehicle = OpenSenseVehicle(simulator_config, vehicle_config)
    #
    # gui = None


    # while episodes > 0:
    #     helper = InterruptHelper()
    #
    #     simulator.restart_event.clear()
    #     simulator.send_vehicle_configuration(vehicle_config)
    #     logging.getLogger("simulator").info('Starting vehicle')
    #     #ego_vehicle.update_fmcw_in_config()
    #     ego_vehicle.start_sensor_streaming(client)
    #     ego_vehicle.start_sensor_listening()
    #
    #     gui = GUI(ego_vehicle, simulator)
    #
    #     #don't need this because we are not driving here yet
    #     #ego_vehicle.init_vehicle_loop(client)
    #
    #     # Waits for the restart event to be set in the control process
    #     time.sleep(100)
    #
    #     ego_vehicle.stop()
    #     #helper.wait(simulator.restart_event)
    #     simulator.restart_event.wait()
    #
    #     gui.stop()

        # Terminates vehicle and sensor processes
    simulator.stop()

    logging.getLogger("simulator").info("episode complete")
    time.sleep(1)

#        episodes = episodes - 1

    logging.getLogger("simulator").info("Good Bye!")