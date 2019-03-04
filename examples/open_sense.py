__version__ = "1.0"


import json
import logging
import os
import time

try:
    import prctl
except:
    pass

from monodrive import SimulatorConfiguration, VehicleConfiguration, Simulator
from monodrive.networking.messaging import ConfigureSensorsCommand, ConfigureTrajectoryCommand, StepSimulationCommand, EgoControlCommand
from monodrive.vehicles import OpenSenseVehicle
from monodrive.vehicles import TeleportVehicle
from monodrive.ui import GUI

from monodrive.networking.client import Client

ManualDriveMode = True


if __name__ == "__main__":

    # Simulator configuration defines network addresses for connecting to the simulator and material properties
    simulator_config = SimulatorConfiguration('simulator.json')

    # Vehicle configuration defines ego vehicle configuration and the individual sensors configurations
    vehicle_config = VehicleConfiguration('demo.json')

    client = Client((simulator_config.configuration["server_ip"],
                     simulator_config.configuration["server_port"]))
    #connect to client
    if not client.isconnected():
        client.connect()

    #set up simulator and send configuration
    simulator = Simulator(client, simulator_config)
    simulator.setup_logger()
    simulator.send_configuration()
    time.sleep(2)

    configPath = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                              "..", "configurations", "open_sense")

    # load sensors
    sensors = vehicle_config.sensor_configuration
    print(client.request(ConfigureSensorsCommand(sensors)))

    # load simulation
    # trajectory = json.load(open(os.path.join(configPath, "LaneChangeReplay.json"), "r"))
    trajectory = json.load(open(os.path.join(configPath, "LeftTurnCrossWalk.json"), "r"))
    client.request(ConfigureTrajectoryCommand(trajectory))

    vehicle = OpenSenseVehicle(simulator_config, vehicle_config)

    gui = GUI(vehicle, simulator)
    time.sleep(5)

    vehicle.start_sensor_listening()
    #vehicle.init_vehicle_loop(client)
    #
    init = True
    # step simulation
    for i in range(0, len(trajectory)):
        if i % 10 == 0:
            print("Processing Trajectory {0}".format(i))
        resp = client.request(StepSimulationCommand(1))
        if resp is None:
            logging.getLogger("control").error(
                "Failed response from sending control data")
        time.sleep(.2)

    time.sleep(10)

    # Terminates vehicle and sensor processes
    vehicle.stop()
    simulator.stop()
    gui.stop()

    logging.getLogger("simulator").info("episode complete")
    time.sleep(1)

    logging.getLogger("simulator").info("Good Bye!")