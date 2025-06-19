from dataclasses import dataclass
import random
import sys
from typing import List, Dict, Set, Tuple, Optional, Callable
from collections import defaultdict
import copy
from carla_utils.carla_env import init_carla_env
init_carla_env()
import carla
import math
import numpy as np
import tqdm
from .utils import batch_destroy, transform_save_utils
from distributions.distribution_metric import distribution_fitting_test
from scipy.stats import beta, rv_continuous,kstest

@dataclass
class LaneSegment:
    """Data class for the lane segment"""
    road_id: int
    lane_id: int
    s_min: float
    s_max: float

    def __hash__(self):
        return hash((self.road_id, self.lane_id, self.s_min, self.s_max))
    
class Semente_helper:
    def __init__(self, lane_segments):
        self.lane_segments = lane_segments
        self._set = set()
        self.s_values_dict: Dict[Tuple[int, int], Set[float]] = dict()
        self.s_values_empty: Dict[Tuple[int, int], List[int]] = dict()
        self.s_values_success: Dict[Tuple[int, int], bool] = defaultdict(lambda: False) 
        self.lanesegment_success_transform : Dict[LaneSegment, List[carla.Transform]] = defaultdict(lambda: []) 
        pass

    def checklist_lanesegment(self, lane_segment):
        """indicating whether the lane segments have been used before"""
        if (lane_segment.road_id, lane_segment.lane_id) not in self._set:
            self._set.add((lane_segment.road_id, lane_segment.lane_id))
            return True
        else:
            return False

    def clear_set(self):
        """clear the current _set, reset for next iteration"""
        self._set.clear()

    def search_s(self, lane_segment, s):
        """Search for offset from the ininital points that has never been touched before. 
        The three loops inside show that we see whehter for some valid (road_id, lane_id),
        there are left available offsets s in the dictironary
        """
        key = (lane_segment.road_id, lane_segment.lane_id)
        s_range = range(math.floor(lane_segment.s_min), math.ceil(lane_segment.s_max))
        
        if key not in self.s_values_dict:
            self.s_values_dict[key] = set()
            s_empty = [0]*len(s_range)
            self.s_values_empty[key] = s_empty

        if s not in self.s_values_dict[key]:
            self.s_values_dict[key].add(s)
            index = s - math.floor(lane_segment.s_min)
            self.s_values_empty[key][index] = 1
            return s

        if s in self.s_values_dict[key]:
            _l = self.s_values_dict[key]
            all_samples = set(s_range)
            if len(_l) < len(all_samples):
                s = random.choice(list(all_samples - _l))
            else:
                return -1
            return s

    def sample_segment_with_checking(self, lane_segments):
        """ Sample transform points by ierating all unique lane-segments"""
        c_sgments  = copy.deepcopy(lane_segments)
        random.shuffle(c_sgments)
        sampled_list = []
        sampeld_table = []
        for l in c_sgments:
            if self.checklist_lanesegment(l):
                sampled_list.append(l)
            else:
                pass
        
        for _segment in sampled_list:
            s_sample = random.choice(range(math.floor(_segment.s_min), math.ceil(_segment.s_max)))
            s_sample = self.search_s(_segment, s_sample)
            if s_sample > 0:
                sampeld_table.append((_segment, s_sample))
            else:
                continue
        return sampeld_table

    def add_leftpver_s(self, lane_segments):
        s_vd = self.s_values_dict
        s_ve = self.s_values_empty

        leftovers = []
        for l in lane_segments:
            k = (l.road_id, l.lane_id)
            empty_list = s_ve[k]
            zero_indices = [i for i, val in enumerate(empty_list) if val == 0]
            for z in zero_indices:
                leftovers.append((l, l.s_min + z))
            
        return leftovers


def try_spawn_from_s(_s, carla_map, client, world, vehicle_bp, success_transforms, spawned_vehicles, success_count, seme_helper):
    """Attempt to spawn a vehicle by the s in a world"""
    try:
        segment, s_sample = _s[0], _s[1]
        wp = carla_map.get_waypoint_xodr(road_id=segment.road_id, lane_id=segment.lane_id, s=s_sample)
        # perturbed_transform = sample_offset_transform(wp)
        spawn_loc = wp.transform.location
        spawn_loc.z += 1
        spawn_transform = carla.Transform(spawn_loc, wp.transform.rotation)
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_transform)#wp.transform
        if vehicle is not None:
            print(f"✅ Spawned at road {segment.road_id}, lane {segment.lane_id}, s={s_sample:.2f}")
            success_transforms.append(wp.transform)
            spawned_vehicles.append(vehicle)
            success_count += 1
            seme_helper.s_values_success[(segment.road_id, segment.lane_id)] = True
            seme_helper.lanesegment_success_transform[segment].append(wp.transform)
            batch_destroy(client=client, world=world, interval=0.2)
        else:
            pass
            print(f"❌ Failed to spawn at road {segment.road_id}, lane {segment.lane_id}, s={s_sample:.2f}")
    except RuntimeError as e:
        print(f"❌ Exception at road={segment.road_id}, lane={segment.lane_id}, s={s_sample:.2f}: {e}")

def save_config(save_file, save_mode):
    def decorator(func):
        func.save_file = save_file
        func.save_mode = save_mode
        return func
    return decorator

def sample_offset_transform(wp):
    
    # Sample lateral offset T and yaw offset W
    T = 0.5 * np.random.beta(2, 2) - 0.25  # range [-0.25, 0.25] meters
    W = 7.2 * np.random.beta(2, 2) - 3.6   # range [-3.6°, 3.6°]

    base_transform = wp.transform
    location = base_transform.location
    rotation = base_transform.rotation

    # Compute right vector from yaw
    yaw_rad = np.radians(rotation.yaw)
    right_vector = carla.Vector3D(x=-np.sin(yaw_rad), y=np.cos(yaw_rad))

    # Apply lateral offset
    location += right_vector * T

    # Apply small yaw rotation offset
    rotation.yaw += W

    return carla.Transform(location, rotation), T, W

def try_spawn_beta_perturb_transform(client, world, carla_map, vehicle_bp, seme_helper, budget = 10000):
    """Within the budget 10000, perturb many groups of start location of the vehicles by a sample_offset_transform method
    Todo: change sample_offset into an method compatible to custmoized distributions.
    """
    all_transforms = []
    for k, transform_list in seme_helper.lanesegment_success_transform.items(): # k every Lanesegment
        for sp in transform_list: # the sucessful transforms in k
            T_list, W_list, local_transforms = [], [], []
            for i in tqdm.tqdm(range(0, budget)):
                perturbed_transform, T, W = sample_offset_transform(carla_map.get_waypoint(sp.location))
                # print(sp, perturbed_transform)
                spawn_loc = perturbed_transform.location
                spawn_loc.z += 1
                spawn_transform = carla.Transform(spawn_loc, perturbed_transform.rotation)
                vehicle = world.try_spawn_actor(vehicle_bp, perturbed_transform)#wp.transform
                try:
                    if vehicle is not None:
                        print(f"✅ perturbed at road", perturbed_transform.location)
                        local_transforms.append(perturbed_transform)
                        T_list.append(T)
                        W_list.append(W)
                        batch_destroy(client=client, world=world, interval=0.2)
                    else:
                        batch_destroy(client=client, world=world, interval=0.2)
                        pass
                        # print(f"[{i}] ❌ Failed to spawn at road {segment.road_id}, lane {segment.lane_id}, s={s_sample:.2f}")
                except RuntimeError as e:
                    print(f"Runtime terror exception")
            print(len(T_list))
            kl_t = distribution_fitting_test(np.asarray(T_list), beta, {"alpha":2, "beta":2},  kstest)
            kl_w = distribution_fitting_test(np.asarray(W_list), beta, {"alpha":2 , "beta":2}, kstest)
            print(kl_t, kl_w)
            local_transforms.append(sp)
            all_transforms.append(local_transforms)
            sys.exit()
    return all_transforms

@save_config(["beta_transform1.json", "beta_transform2.json", "perturb_transform.json"],(1,2,3))
def try_spawn_from_lane_segments(client: carla.Client, lane_segments: List[LaneSegment], num_attempts: int = 30) -> List[carla.Actor]:
    import time
    world = client.get_world()
    carla_map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.tesla.model3')

    spawned_vehicles = []
    success_transforms = []
    seme_helper = Semente_helper(lane_segments)
    random.shuffle(lane_segments)
    
    n, success_count = 0, 0

    """Frit stage of collecting spawnable points from different scenes"""
    while n < 1:
        sampled_table = seme_helper.sample_segment_with_checking(lane_segments)
        for _s in tqdm.tqdm(sampled_table):
            try_spawn_from_s(_s, carla_map, client, world, vehicle_bp, success_transforms, spawned_vehicles, success_count, seme_helper)
        batch_destroy(client=client, world=world, interval = 1)
        seme_helper.clear_set()
        n += 1
    print(len(success_transforms))
    transform_save_utils(try_spawn_from_lane_segments.save_file[0], try_spawn_from_lane_segments.save_mode, 1,  success_transforms)
    
    all_transofrms = try_spawn_beta_perturb_transform(client, world, carla_map, vehicle_bp, seme_helper, budget = 1000)
    print(all_transofrms)
    transform_save_utils(try_spawn_from_lane_segments.save_file[2], try_spawn_from_lane_segments.save_mode, 3,  all_transofrms)

