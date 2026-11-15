"""Replace the static hatchback in the open world with the tested physical car."""
from datetime import datetime
from pathlib import Path
import os
import signal
import subprocess
import time

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.sdf_generator_config_pb2 import SdfGeneratorConfig
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.pose_pb2 import Pose
from car_controls import Car

ROOT = Path(__file__).resolve().parent
node = Node()


def request(service, message, response_type=Boolean):
    ok, result = node.request('/world/shapes/'+service, message, type(message), response_type, 5000)
    if not ok or (response_type is Boolean and not result.data):
        raise RuntimeError('Gazebo service failed: '+service)
    return result


def save_world(path):
    config = SdfGeneratorConfig()
    config.global_entity_gen_config.expand_include_tags.data = True
    config.global_entity_gen_config.resources_use_absolute_paths.data = True
    result = request('generate_world_sdf', config, StringMsg)
    path.write_text(result.data)
    print('Saved world:',path,flush=True)


def main():
    # Close only this project's old controller, whose pose commands bypass physics.
    controller = str(ROOT/'car_controls.py').encode()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name)==os.getpid():
            continue
        try:
            arguments=(entry/'cmdline').read_bytes().split(b'\0')
            if controller in arguments and arguments[0].endswith(b'python3'):
                os.kill(int(entry.name),signal.SIGTERM)
                print('Closed old controller:',entry.name,flush=True)
        except (FileNotFoundError,ProcessLookupError,PermissionError):
            pass
    time.sleep(.3)
    car=Car()
    pose=Pose()
    pose.CopyFrom(car.refresh())
    print('Preserving car location:',pose.position,flush=True)
    backups=ROOT/'backups'
    backups.mkdir(exist_ok=True)
    save_world(backups/('before_physics_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'.sdf'))
    request('remove/blocking',Entity(name='hatchback_blue',type=Entity.MODEL))
    replacement=EntityFactory(name='hatchback_blue',
                              sdf_filename=str(ROOT/'models/hatchback_drivable/model.sdf'))
    replacement.pose.CopyFrom(pose)
    replacement.pose.ClearField('id')
    replacement.pose.ClearField('name')
    replacement.pose.position.z=max(pose.position.z,.025)
    try:
        request('create/blocking',replacement)
    except Exception:
        replacement.sdf_filename=str(Path.home()/'.gz/fuel/fuel.gazebosim.org/openrobotics/models/hatchback%20blue/5/model.sdf')
        request('create/blocking',replacement)
        raise
    time.sleep(1)
    if not car.connected():
        raise RuntimeError('New vehicle controller was not discovered.')
    car.command()
    print('Physical car controller connected.',flush=True)
    print('Current pose:',car.refresh(),flush=True)
    save_world(ROOT/'world_drivable.sdf')
    with open('/tmp/gazebot-car-controls.log','a') as log:
        process=subprocess.Popen(['python3',str(ROOT/'car_controls.py')],
            stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    print('Opened upgraded controls:',process.pid,flush=True)


if __name__=='__main__':
    main()
