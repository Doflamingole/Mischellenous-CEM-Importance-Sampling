import sys
from carla_utils import init_carla_env
init_carla_env()
import carla
from carla_utils import extract_valid_lane_segments
from carla_utils import try_spawn_from_lane_segments
from carla_utils import init_carla_env

from time import time
from dataclasses import dataclass
        
def main():
    import sys
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    # world = client.load_world("Town02")
    world = client.get_world()
    carla_map = world.get_map()

    vehicles = world.get_actors().filter("*vehicle*")
    for v in vehicles:
        v.destroy()
    # 提取 LaneSegment 列表
    lane_segments = extract_valid_lane_segments(carla_map)

    # print(len(len_news))

    # 尝试从这些 LaneSegment 中生成车辆
    success_transforms = try_spawn_from_lane_segments(client, lane_segments, num_attempts=30)

    print(len(success_transforms))
    # 停留几秒后销毁
    import time
    time.sleep(5)
    for v in vehicles:
        v.destroy()
    print("✅ Cleaned up all spawned vehicles.")


if __name__ == "__main__":
    main()