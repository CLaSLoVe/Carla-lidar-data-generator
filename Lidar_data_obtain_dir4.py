import numpy as np
import random
import datetime
import cv2
import pickle
import glob
import os
import sys
import time
from client_bounding_boxes import ClientSideBoundingBoxes
from tools import *
from validate_bb import batch_validate


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

RECORD = True
SAVE_PIC = True

THRESHOLD = [10, 8, 5]
TIME = 1207

if not RECORD:
    SAVE_PIC = False


def set_lidar(world, sensor_options, transform, attached):
    lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    lidar_bp.set_attribute('range', '100')

    for key in sensor_options:
        lidar_bp.set_attribute(key, sensor_options[key])
    lidar = world.spawn_actor(lidar_bp, transform, attach_to=attached)

    return lidar


def _on_gnss_event(event, path):
    """GNSS method"""
    lat = event.latitude
    lon = event.longitude
    alt = event.altitude
    timestamp = event.timestamp
    array = np.array([lon, lat, alt])
    name = time2name(event.timestamp)
    if RECORD:
        np.savetxt(path + '/' + name + '.txt', array)


def only_xyzyaw(event, vehicle, path):
    name = time2name(event.timestamp)
    x = vehicle.get_transform().location.x
    y = vehicle.get_transform().location.y
    z = vehicle.get_transform().location.z
    yaw = np.radians(vehicle.get_transform().rotation.yaw)
    array = np.array([x, y, z, yaw])
    if RECORD:
        np.savetxt(path + '/' + name + '.txt', array)


def save_pic(image, path):
    image.convert(carla.ColorConverter.Raw)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    if RECORD:
        name = time2name(image.timestamp)
        cv2.imwrite(path + '/' + name + '.jpg', array)


def run_simulation(client, t=1200):
    """This function performed one test run using the args parameters
    and connecting to the carla client passed.
    """
    try:
        world = client.get_world()
        original_settings = world.get_settings()
        tm_port = 8005

        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.1
        world.apply_settings(settings)
        actors = []
        actors.extend(world.get_actors().filter('vehicle.*'))
        actors.extend(world.get_actors().filter('walker.*'))

        # put car
        bp = world.get_blueprint_library().filter('model3')[0]
        vehicle = None
        while not vehicle:
            # vehicle = world.try_spawn_actor(bp, random.choice(world.get_map().get_spawn_points()))
            vehicle = world.try_spawn_actor(bp, world.get_map().get_spawn_points()[3])
        vehicle.set_autopilot(True, tm_port)

        # wait to initialize
        time.sleep(1)

        # put sensors
        sensor_tick = '0.1'
        par = {'sensor_tick': sensor_tick, 'range': '150', 'points_per_second': '576000',
           'rotation_frequency': '10'}
        Lidars = []
        Lidars.append(set_lidar(world, par, carla.Transform(carla.Location(x=vehicle.bounding_box.extent.x, z=0.4)), vehicle))
        Lidars.append(set_lidar(world, par, carla.Transform(carla.Location(x=vehicle.bounding_box.extent.x, z=0.8)), vehicle))
        Lidars.append(set_lidar(world, par, carla.Transform(carla.Location(x=vehicle.bounding_box.extent.x, z=1.6)), vehicle))
        output_dir = ''

        def gen_lidar_dir(h):
            os.mkdir('data/' + output_dir + '/' + str(h))
            os.mkdir('data/' + output_dir + '/' + str(h) + '/label')
            os.mkdir('data/' + output_dir + '/' + str(h) + '/lidar')
            os.mkdir('data/' + output_dir + '/' + str(h) + '/calib')
            os.mkdir('data/' + output_dir + '/' + str(h) + '/posit')

        # record prepare
        if RECORD:
            output_dir = datetime.datetime.now().strftime('%Y-%m-%d %H.%M.%S')
            os.mkdir('data/' + output_dir)
            os.mkdir('data/' + output_dir + '/GPS')
            # os.mkdir('data/' + output_dir + '/actor')

            gen_lidar_dir(0.4)
            gen_lidar_dir(0.8)
            gen_lidar_dir(1.6)

        if SAVE_PIC:
            os.mkdir('data/' + output_dir + '/RGB')
            os.mkdir('data/' + output_dir + '/0.8/pics')
            os.mkdir('data/' + output_dir + '/0.4/pics')
            os.mkdir('data/' + output_dir + '/1.6/pics')

        # generate lidars!
        world_par = {'actors': actors, 'sensor_par': par}
        Lidars[0].listen(lambda data: save_lidar_image(Lidars[0], world_par, data, path='data/' + output_dir + '/0.4'))
        Lidars[1].listen(lambda data: save_lidar_image(Lidars[1], world_par, data, path='data/' + output_dir + '/0.8'))
        Lidars[2].listen(lambda data: save_lidar_image(Lidars[2], world_par, data, path='data/' + output_dir + '/1.6'))

        # generate gps..
        # gnss_bp = world.get_blueprint_library().find('sensor.other.gnss')
        # gnss = world.spawn_actor(gnss_bp, carla.Transform(carla.Location(x=0, z=0.0)), attach_to=vehicle)
        # gnss.listen(lambda event: _on_gnss_event(event, path='data/' + output_dir + '/GPS'))

        xyz_bp = world.get_blueprint_library().find('sensor.other.gnss')
        gnss = world.spawn_actor(xyz_bp, carla.Transform(carla.Location(x=0, z=0.0)), attach_to=vehicle)
        gnss.listen(lambda event: only_xyzyaw(event, vehicle, path='data/' + output_dir + '/GPS'))

        if SAVE_PIC:
            # generate rgb..
            rgb_bp = world.get_blueprint_library().find('sensor.camera.rgb')
            rgb = world.spawn_actor(rgb_bp, carla.Transform(carla.Location(z=120), carla.Rotation(pitch=-90)), attach_to=vehicle)
            rgb.listen(lambda image: save_pic(image, path='data/' + output_dir + '/RGB'))

        # main loop
        init_frame = world.get_snapshot().frame
        world_frame = 0
        while world_frame <= t:
            # Carla Tick
            world.wait_for_tick()
            ego_transform = vehicle.get_transform()

            spectator = world.get_spectator()
            spectator.set_transform(carla.Transform(ego_transform.location + carla.Location(z=80),
                                                    carla.Rotation(pitch=-90, yaw=ego_transform.rotation.yaw)))

            world.wait_for_tick()
            world_frame = world.get_snapshot().frame - init_frame
            print(world_frame)
        # print(batch_validate(PATH))
    finally:

        client.apply_batch([carla.command.DestroyActor(x) for x in [vehicle]])
        print('\ndestroying %d vehicles' % 1)

        world.apply_settings(original_settings)
        sys.exit(3)

def save_lidar_image(lidar, world_par, image, path=None):
    lidar.timestamp = image.timestamp
    lidar_range = float(world_par['sensor_par']['range']) * 2
    lidar_matrix = image.transform.get_matrix()
    # print(lidar.get_transform())

    actor_dict = {}
    for actor_id, actor in enumerate(world_par['actors']):
        actor_dict[actor_id] = {}
        actor_dict[actor_id]['yaw'] = actor.get_transform().rotation.yaw
        actor_dict[actor_id]['l'] = actor.bounding_box.extent.x
        actor_dict[actor_id]['w'] = actor.bounding_box.extent.y
        actor_dict[actor_id]['h'] = actor.bounding_box.extent.z
        actor_dict[actor_id]['x'] = actor.get_transform().location.x
        actor_dict[actor_id]['y'] = actor.get_transform().location.y
        actor_dict[actor_id]['z'] = actor.get_transform().location.z
        actor_dict[actor_id]['type'] = str(actor.type_id)
        actor_dict[actor_id]['valid'] = 0


    points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 4), 4))
    local_lidar_points = np.array(points[:, :3])

    if SAVE_PIC:
        pix = 1000
        lidar_img_size = (pix, pix, 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
        lidar_data = np.array([-points[:, 0], points[:, 1]]).T
        lidar_data *= pix / lidar_range
        lidar_data += (0.5 * pix, 0.5 * pix)
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img[tuple(lidar_data.T)] = (205, 205, 205)

        r1 = int(50 * pix / lidar_range)
        r2 = int(100 * pix / lidar_range)
        center = np.array([0.5 * pix, 0.5 * pix]).astype(np.int32)
        cv2.circle(lidar_img, center, r1, (0, 0, 255))
        cv2.circle(lidar_img, center, r2, (0, 0, 255))

    # ego_transform = lidar.transform()
    ego_transform = image.transform
    world_2_sensor = np.array(ego_transform.get_inverse_matrix())



    labels = []
    for actor_id, actor in enumerate(world_par['actors']):

        bb_cords = ClientSideBoundingBoxes._create_bb_points(actor)
        world_cord = ClientSideBoundingBoxes._vehicle_to_world(bb_cords, actor)
        bb_points = np.array(world_cord.T).reshape(-1, 4).T
        bb_sensor = np.dot(world_2_sensor, bb_points).T

        points_draw = np.array([-bb_sensor[:, 0], bb_sensor[:, 1]]).T
        points_draw *= pix / lidar_range
        points_draw += (0.5 * pix, 0.5 * pix)
        points_draw = points_draw.astype(np.int32).T
        x = (points_draw[0])
        y = (points_draw[1])

        if SAVE_PIC:
            for i in range(4):
                cv2.line(lidar_img, (int(y[i]), int(x[i])), (int(y[i + 1]), int(x[i + 1])), (0, 100, 100), 1, 4)

        actor_dict[actor_id]['bb'] = bb_sensor
        actor_dict[actor_id]['draw'] = [x, y]
        # bb_sensor = bb_sensor[:, :3]
        # Oz = (bb_sensor[0] + bb_sensor[2]) / 2
        # Ox = (bb_sensor[5] + bb_sensor[2]) / 2
        # Oy = (bb_sensor[5] + bb_sensor[0]) / 2
        # O  = (bb_sensor[6] + bb_sensor[0]) / 2
        #
        # Px = unit_vector(Ox - O)
        # Py = unit_vector(Oy - O)
        # Pz = unit_vector(Oz - O)
        # #
        # actor_co = np.array([Px, Py, Pz]).T
        # vecs = local_lidar_points - O
        # p = np.dot(vecs, actor_co)
        # p = np.fabs(p)
        # l = shape.x  # *1.15
        # w = shape.y  # *1.15
        # h = shape.z  # *1.15
        # valid_array = np.logical_and(np.logical_and(p[:, 0] <= l, p[:, 1] <= w), p[:, 2] <= h)
        # ans = np.sum(valid_array)
        # L = np.sqrt(O[0]**2+O[1]**2)
        #
        #
        # if L < 50:
        #     v = THRESHOLD[0]
        # elif L < 100:
        #     v = THRESHOLD[1]
        # else:
        #     v = THRESHOLD[2]
        #
        # if actor_dict[actor]['type'][0] == 'w':
        #     v = v / 2
        #     if ans >= v:
        #         actor_dict[actor]['valid'] = 1
        # else:
        #     if ans >= v:
        #         actor_dict[actor]['valid'] = 1
        #
        # if actor_dict[actor]['valid']:
        #     if O[0]-ego_transform.location.x == 0:
        #         absolute_actor_yaw = 90
        #     else:
        #         absolute_actor_yaw = np.arctan((O[1]-ego_transform.location.y)/(O[0]-ego_transform.location.x))
        #     alpha = degree2radian(np.degrees(absolute_actor_yaw - image.transform.rotation.yaw))
        #     yaw = degree2radian(actor.get_transform().rotation.yaw - image.transform.rotation.yaw)
        #     actor_vec = np.array([(O[0]-ego_transform.location.x), (O[1]-ego_transform.location.y)])
        #     Lxy = np.sqrt(actor_vec[0]**2+actor_vec[1]**2)
        #     dx = Lxy*np.cos(alpha)
        #     dy = Lxy*np.sin(alpha)
        #     dz = O[2]-ego_transform.location.z
        #     string = [classify_actor(actor_dict[actor]),
        #               1,
        #               1,
        #               alpha,
        #               1,
        #               1,
        #               1,
        #               1,
        #               h * 2,
        #               w * 2,
        #               l * 2,
        #               dx,
        #               dy,
        #               dz,
        #               yaw,
        #               1]
        #     labels.append(string)
        #     if SAVE_PIC:
        #         for i in range(4):
        #             cv2.line(lidar_img, (int(y[i]), int(x[i])), (int(y[i + 1]), int(x[i + 1])), (0, 255, 0), 1, 4)
    posit = {}
    posit['x'] = ego_transform.location.x
    posit['y'] = ego_transform.location.y
    posit['z'] = ego_transform.location.z
    posit['yaw'] = ego_transform.rotation.yaw

    posit['dict'] = actor_dict

    if RECORD:
        name = time2name(image.timestamp)
        points.tofile(path + '/lidar/' + name + '.bin')
        np.savetxt(path + '/label/' + name + '.txt', labels, fmt='%s')
        np.savetxt(path + '/calib/' + name + '.txt', lidar_matrix)
        with open(path + '/posit/' + name + '.pkl', 'wb') as f:
            pickle.dump(posit, f)
    if SAVE_PIC:
        cv2.imwrite(path + '/pics/' + name + '.png', lidar_img)

def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(5.0)

        run_simulation(client, t=TIME)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
