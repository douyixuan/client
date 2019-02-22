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
from monodrive.networking.messaging import ConfigureSensorsCommand, ConfigureTrajectoryCommand, StepSimulationCommand

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

    configPath = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                              "..", "configurations", "open_sense")

    # load sensors
    sensors = json.load(open(os.path.join(configPath, "sensors.json"), "r"))
    print(client.request(ConfigureSensorsCommand(sensors)))

    # load simulation
    trajectory = json.load(open(os.path.join(configPath, "Trajectory.json"), "r"))
    print(client.request(ConfigureTrajectoryCommand(trajectory)))

    # step simulation
    for i in range(0, len(trajectory)):
        print(client.request(StepSimulationCommand(1)))
        time.sleep(2)

    # Terminates vehicle and sensor processes
    simulator.stop()

    logging.getLogger("simulator").info("episode complete")
    time.sleep(1)

    logging.getLogger("simulator").info("Good Bye!")