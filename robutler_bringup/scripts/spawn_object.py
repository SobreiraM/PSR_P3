#!/usr/bin/env python3

import random
from random import randint
import uuid
from colorama import Fore, Style
import rospy
import rospkg
import argparse
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


# TODO
# argparse for mode of Spawn
# functions for each mode
# add models
# create function to generate models? 

def arg_function():
    """
    Function to parse arguments from command line.
    """
    parser = argparse.ArgumentParser(description='Definições de spawn de objetos')
    parser.add_argument("-o", "--object", required=True,  type= str, default='sphere_v',
                        help="Objetos :")
    parser.add_argument('-place','--place_to_spawn', required=True,  type= str, default='bed',
                        help='Lugar para spawn do objeto: bed, bedroom_table, bedroom_chair, sofa, orange_table, shelf, under_kitchen_table, door')
    parser.add_argument("-rand", "--random_spawn",
                        required=False, action="store_true",
                        help="Ativar o modo de spawn aleatório de objetos.")
    
    return parser.parse_args()


def Set_Spawn_Points():
    
     # Setting available spawn points
    spawn_points = {}

    p = Pose()                   
    q = quaternion_from_euler(0,0,0)    # var q and orientation will be the same for every location
    
    # Created different vars for Pose() because the last spawnpoint pose was being used for every spawnpoint
    # On bed
    p1 = Pose()
    p1.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p1.position = Point(x=-6.033466, y=1.971232, z=0.644345) #coordinates of the location
    spawn_points['bed'] = {'pose':p1}

    # On bedroom table
    p2 = Pose()
    p2.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p2.position = Point(x=-8.910127, y=1.585621, z=0.744070)
    spawn_points['bedroom_table'] = {'pose': p2}

    # On bedroom chair
    p3 = Pose()
    p3.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p3.position = Point(x=-8.247623, y=-4.505759, z=0.360569)
    spawn_points['bedroom_chair'] = {'pose': p3}

    # On sofa
    p4 = Pose()
    p4.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p4.position = Point(x=-0.289190, y=-1.449864, z=0.478602)
    spawn_points['sofa'] = {'pose': p4}

    # On orange table
    p5 = Pose()
    p5.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p5.position = Point(x=-0.595728, y=4.076783, z=0.399902)
    spawn_points['orange_table'] = {'pose': p5}

    # On shelf
    p6 = Pose()
    p6.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p6.position = Point(x=4.321983, y=-5.099168, z=0.383039)
    spawn_points['shelf'] = {'pose': p6}

    # At the door
    p7 = Pose()
    p7.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p7.position = Point(x=5.989245, y=-5.032764, z=-0.019552)
    spawn_points['door'] = {'pose': p7}

    # Under kitchen table
    p8 = Pose()
    p8.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    p8.position = Point(x=6.347103, y=1.001381, z=-0.019552)
    spawn_points['under_kitchen_table'] = {'pose': p8}

    return spawn_points


def Set_Objects():

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_description') + '/models/' 
    objects = {}

    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['sphere_v'] = {'name': 'sphere_v', 'sdf': f.read()} 

    f = open(package_path + 'person/model.sdf', 'r')
    objects['person'] = {'name': 'person', 'sdf': f.read()}

    f = open(package_path + 'coke_can/model.sdf', 'r')
    objects['coke_can'] = {'name': 'coke_can', 'sdf': f.read()}

    return objects


def main():

    args = vars(arg_function())

    spawn_points = Set_Spawn_Points()
    objects = Set_Objects()
    

    if not args['place_to_spawn'] in spawn_points.keys():
        print('Place ' + args['place_to_spawn'] + 'is unknown or not defined. Available options are ' + 
            str(list(spawn_points.keys())))
    
    if not args['object'] in objects.keys():
        print('Object ' + args['object'] + 'is unknown or not defined. Available options are ' + 
            str(list(objects.keys())))

    rospy.init_node('insert_object', log_level=rospy.INFO)

    service_name = '/gazebo/spawn_sdf_model'
    rospy.wait_for_service(service_name)
    service_client = rospy.ServiceProxy(service_name, SpawnModel)

    if args['random_spawn']:
        n_objects = randint(1, 8)
        print(n_objects)
        print('Spawning objects at these locations: ')
        for spawn in range(n_objects):
            uuid_str = str(uuid.uuid4())
            random_place = random.choice(list(spawn_points.keys()))
            print(random_place)

            service_client(objects[args['object']]['name'] + '_' + uuid_str,
                        objects[args['object']]['sdf'],
                        objects[args['object']]['name'] + '_' + uuid_str,
                        spawn_points[random_place]['pose'],
                        'world')   
    else: 
        print('Spawning an object... ')
        uuid_str = str(uuid.uuid4())

        service_client(objects[args['object']]['name'] + '_' + uuid_str,
                    objects[args['object']]['sdf'],
                    objects[args['object']]['name'] + '_' + uuid_str,
                    spawn_points[args['place_to_spawn']]['pose'],
                    'world')

if __name__ == '__main__':
    main()