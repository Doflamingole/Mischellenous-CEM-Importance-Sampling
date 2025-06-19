import json
import time
from carla_utils.carla_env import init_carla_env
init_carla_env()
import carla

def transform_to_dict(transform):
    return {
        'location': {
            'x': transform.location.x,
            'y': transform.location.y,
            'z': transform.location.z
        },
        'rotation': {
            'pitch': transform.rotation.pitch,
            'yaw': transform.rotation.yaw,
            'roll': transform.rotation.roll
        }
    }

def save_transforms_to_json(transform_list, filename='perturb_transform.json'):
    transform_dicts = [transform_to_dict(t) for t in transform_list]
    with open(filename, 'w') as f:
        json.dump(transform_dicts, f, indent=4)

def batch_destroy(client, world, interval):
    """Batch destroy vechilce on the map for the next generation of spawning"""
    vehicles = world.get_actors().filter('vehicle.*')
    destroy_commands = [carla.command.DestroyActor(vehicle.id) for vehicle in vehicles]

    # 批量执行并同步，确保立即生效
    client.apply_batch_sync(destroy_commands, True)
    world.tick()
    time.sleep(interval)

def transform_save_utils(save_file, save_mode, index, success_transforms):
    if index in save_mode:
        transform_dicts = [transform_to_dict(t) for t in success_transforms]
        with open(save_file, 'a') as f:
            json.dump(transform_dicts, f, indent=4)

