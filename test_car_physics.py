"""Integration check using disposable models away from the user's scene."""
from pathlib import Path
import math
import time
import uuid
from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.entity_pb2 import Entity
from car_controls import Car

node = Node()
world = 'shapes'
prefix = 'gazebot_test_' + uuid.uuid4().hex[:8]
created = []


def request(service, message):
    ok, reply = node.request(f'/world/{world}/{service}', message, type(message), Boolean, 5000)
    if not ok or not reply.data:
        raise RuntimeError(f'{service} failed')


def spawn(name, x, y, sdf='', path=''):
    message = EntityFactory(name=name, allow_renaming=False)
    if path:
        message.sdf_filename = path
    else:
        message.sdf = sdf
    message.pose.position.x, message.pose.position.y = x,y
    message.pose.position.z = 0.025
    message.pose.orientation.w = 1
    request('create/blocking', message)
    created.append(name)


def drive(car, velocity, angle, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        car.command(velocity, angle)
        time.sleep(.05)
    car.command()
    time.sleep(.6)
    return car.refresh()


def yaw(pose):
    q=pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


car = None
try:
    name = prefix+'_car'
    spawn(name,30,30,path=str(Path(__file__).parent/'models/hatchback_drivable/model.sdf'))
    car = Car(world,name)
    time.sleep(2)
    print('Controller connected:',car.connected(),flush=True)
    assert car.connected(), 'Ackermann controller did not load'
    start = car.refresh()
    x0,y0=start.position.x,start.position.y
    print('Start:',x0,y0,start.position.z,flush=True)
    forward=drive(car,1,0,2)
    print('Forward:',forward.position.x,forward.position.y,forward.position.z,flush=True)
    assert forward.position.x>x0+.5,'Forward travel failed'
    xf=forward.position.x
    backward=drive(car,-1,0,2)
    print('Backward:',backward.position.x,backward.position.y,flush=True)
    assert backward.position.x<xf-.5,'Reverse travel failed'
    a0=yaw(backward)
    left=drive(car,1,.45,2.5)
    a1=yaw(left)
    print('Left yaw change:',a1-a0,flush=True)
    assert a1-a0>.15,'Left turn failed'
    right=drive(car,1,-.45,2.5)
    a2=yaw(right)
    print('Right yaw change:',a2-a1,flush=True)
    assert a2-a1<-.15,'Right turn failed'
    print('PASS: forward, backward, left, and right',flush=True)
    car.command()
    name2=prefix+'_collision_car'
    spawn(name2,30,20,path=str(Path(__file__).parent/'models/hatchback_drivable/model.sdf'))
    collision_car=Car(world,name2)
    wall='''<sdf version="1.9"><model name="wall"><static>true</static><link name="link">
    <pose>0 0 1 0 0 0</pose><collision name="wall"><geometry><box><size>.5 8 2</size></box></geometry></collision>
    <visual name="wall"><geometry><box><size>.5 8 2</size></box></geometry></visual></link></model></sdf>'''
    spawn(prefix+'_wall',35,20,sdf=wall)
    time.sleep(1)
    final=drive(collision_car,1,0,5)
    print('Wall test x:',final.position.x,flush=True)
    assert 31<final.position.x<32.6,'Car did not stop against the collision wall'
    print('PASS: wall blocks the car',flush=True)
    push_name=prefix+'_push_car'
    spawn(push_name,30,10,path=str(Path(__file__).parent/'models/hatchback_drivable/model.sdf'))
    push_car=Car(world,push_name)
    cube='''<sdf version="1.9"><model name="cube"><link name="link"><pose>0 0 .5 0 0 0</pose>
    <inertial><mass>5</mass><inertia><ixx>.8333</ixx><iyy>.8333</iyy><izz>.8333</izz></inertia></inertial>
    <collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
    <visual name="visual"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>'''
    cube_name=prefix+'_cube'
    spawn(cube_name,34,10,sdf=cube)
    time.sleep(1)
    drive(push_car,.8,0,4)
    cube_pose=Car(world,cube_name).refresh()
    print('Pushed cube x:',cube_pose.position.x,flush=True)
    assert cube_pose.position.x>34.3,'Car did not push the dynamic cube'
    print('PASS: dynamic object collision and pushing',flush=True)
finally:
    if car:
        car.command()
    for name in reversed(created):
        try:
            request('remove/blocking',Entity(name=name,type=Entity.MODEL))
        except Exception as exc:
            print('Cleanup failed:',name,exc,flush=True)
