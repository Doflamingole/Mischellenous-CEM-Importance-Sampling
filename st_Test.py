import carla
import random
import time

def main():
    # Connect to CARLA
    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    map = world.get_map()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = random.choice(blueprint_library.filter("vehicle.*"))

    # === Step 1: Generate lane segments ===
    # Each waypoint is the center of a lane segment, sampled every 2 meters
    lane_segments = map.generate_waypoints(distance=2.0)
    print(f"Total lane segments (waypoints) found: {len(lane_segments)}")

    # Optional: filter only driving lanes
    driving_segments = [wp for wp in lane_segments if wp.lane_type == carla.LaneType.Driving]
    print(f"Driving lane segments: {len(driving_segments)}")

    # === Step 2: Try to spawn at each segment ===
    total_attempts = 50
    success_count = 0
    spawned_vehicles = []

    random.shuffle(driving_segments)

    for i, wp in enumerate(driving_segments[:total_attempts]):
        transform = wp.transform  # Includes location and yaw
        vehicle = world.try_spawn_actor(vehicle_bp, transform)
        if vehicle is not None:
            print(f"[{i}] ✅ Spawned at ({transform.location.x:.2f}, {transform.location.y:.2f})")
            success_count += 1
            spawned_vehicles.append(vehicle)
        else:
            print(f"[{i}] ❌ Failed to spawn")

    print(f"\nSummary: {success_count}/{total_attempts} vehicles successfully spawned.")

    # === Optional: Wait and clean up ===
    time.sleep(5)
    for v in spawned_vehicles:
        v.destroy()
    print("Cleaned up all spawned vehicles.")

if __name__ == "__main__":
    main()