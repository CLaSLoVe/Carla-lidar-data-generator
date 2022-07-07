import carla
import random
import time

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
client.set_timeout(5.0)

world = client.load_world('Town02')
# world = client.generate_opendrive_world('OpenDriveMap')
vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)

# Set up the TM in synchronous mode
traffic_manager = client.get_trafficmanager(8005)
traffic_manager.set_synchronous_mode(True)
tm_port = traffic_manager.get_port()


while True:
    world.tick()
    time.sleep(0.03)
