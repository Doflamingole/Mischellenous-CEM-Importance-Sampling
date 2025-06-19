import numpy as np
from typing import List
from .segment_lane import LaneSegment
from carla_utils.carla_env import init_carla_env
init_carla_env()
import carla
def extract_valid_lane_segments(carla_map: carla.Map, waypoint_distance: float = 1.0) -> List[LaneSegment]:
    # Step 1: 获取拓扑中的有效 (road_id, lane_id) 对
    topology = carla_map.get_topology()
    valid_pairs = set()
    for wp_start, wp_end in topology:
        valid_pairs.add((wp_start.road_id, wp_start.lane_id))
        valid_pairs.add((wp_end.road_id, wp_end.lane_id))
    # Step 2: 用 generate_waypoints 遍历所有路径点
    waypoints = carla_map.generate_waypoints(distance=waypoint_distance)

    # Step 3: 收集每个有效 (road_id, lane_id) 对应的 s 值范围
    s_values_dict = {}
    for wp in waypoints:
        key = (wp.road_id, wp.lane_id)
        if key in valid_pairs:
            s_values_dict.setdefault(key, []).append(wp.s)

    lane_segments = []
    for (road_id, lane_id), s_list in s_values_dict.items():
        s_min, s_max = min(s_list), max(s_list)
        lane_segments.append(LaneSegment(road_id, lane_id, s_min, s_max))

    print(f"✅ Extracted {len(lane_segments)} valid LaneSegments")
    return lane_segments

