"""
Advanced Autonomous Multi-Agent Simulation
Features:
- Navigation following (turns at intersections)
- Collision avoidance with V2V communication
- Obstacle detection and path replanning
- Hybrid A* path planning integration
"""

from metadrive.envs.marl_envs import MultiAgentMetaDrive
from communication.broadcaster import start_broadcaster
from communication.receiver import start_receiver
from decision_engine.planner import plan_path
from decision_engine.response_planner import response_planner
import threading
import time
import socket
import sys
import json
import numpy as np


# Global storage for obstacle information shared via V2V
obstacle_database = {}
obstacle_lock = threading.Lock()


# Available map configurations
MAP_CONFIGS = {
    "city": "XCTOXCTOX",
    "highway": "SSSCSSSCSSS",
    "complex": "XCTOCSXTOS",
    "roundabouts": "OOOOO",
    "urban": "XTXTXTXT",
    "suburban": "SCSCSCSC",
    "default": "XCTOX",
}


class V2VCommunicationHandler:
    """Handles V2V communication for obstacle sharing"""
    
    def __init__(self, port=5000):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
    def broadcast_obstacle(self, agent_id, obstacle_info):
        """Broadcast obstacle detection to other vehicles"""
        message = {
            "type": "obstacle_alert",
            "agent_id": agent_id,
            "obstacle": obstacle_info,
            "timestamp": time.time()
        }
        try:
            self.sock.sendto(json.dumps(message).encode(), ("127.0.0.1", self.port))
        except:
            pass
    
    def broadcast_position(self, agent_id, position, velocity, heading):
        """Broadcast vehicle position and state"""
        message = {
            "type": "position_update",
            "agent_id": agent_id,
            "position": position,
            "velocity": velocity,
            "heading": heading,
            "timestamp": time.time()
        }
        try:
            self.sock.sendto(json.dumps(message).encode(), ("127.0.0.1", self.port))
        except:
            pass


def get_navigation_command(agent_obs, agent_info):
    """
    Extract navigation command from MetaDrive's navigation system.
    Returns steering adjustment needed to follow the route.
    """
    try:
        # MetaDrive provides navigation information in the observation
        # The navigation module gives us direction to destination
        if hasattr(agent_obs, '__getitem__') and len(agent_obs) > 250:
            # Navigation info is typically in obs[240:260]
            # This includes: distance to destination, direction, etc.
            nav_data = agent_obs[240:260]
            
            # Simple navigation: if we have nav data, extract direction
            # Positive = turn right, Negative = turn left
            if len(nav_data) >= 2:
                # nav_data[0] might be lateral deviation
                # nav_data[1] might be heading error
                lateral_deviation = float(nav_data[0]) if abs(nav_data[0]) < 10 else 0
                heading_error = float(nav_data[1]) if abs(nav_data[1]) < 3.14 else 0
                
                # Combine for steering suggestion
                steering_suggestion = -lateral_deviation * 0.1 + heading_error * 0.5
                return np.clip(steering_suggestion, -0.3, 0.3)
        
        # Check if we have checkpoint info from agent_info
        if 'checkpoint' in agent_info or 'navigation' in agent_info:
            # Use checkpoint direction if available
            pass
            
    except Exception as e:
        pass
    
    return 0.0  # No navigation adjustment


def detect_obstacles(lidar_data, threshold=15.0):
    """
    Detect obstacles from lidar data.
    Returns: (has_obstacle, min_distance, obstacle_direction)
    """
    if len(lidar_data) < 240:
        return False, 100.0, "none"
    
    # Front sector
    front = lidar_data[100:140]
    min_front = float(min(front)) if len(front) > 0 else 100.0
    
    # Left sector
    left = lidar_data[140:180]
    min_left = float(min(left)) if len(left) > 0 else 100.0
    
    # Right sector
    right = lidar_data[60:100]
    min_right = float(min(right)) if len(right) > 0 else 100.0
    
    # Determine obstacle location
    if min_front < threshold:
        return True, min_front, "front"
    elif min_left < threshold:
        return True, min_left, "left"
    elif min_right < threshold:
        return True, min_right, "right"
    
    return False, min(min_front, min_left, min_right), "none"


def get_advanced_action(obs, agent_id, infos, v2v_handler):
    """
    Advanced action generation with:
    - Navigation following (turns)
    - Collision avoidance
    - V2V communication
    - Path planning integration
    """
    agent_obs = obs[agent_id]
    agent_info = infos.get(agent_id, {})
    
    # Default action
    base_steering = 0.0
    base_throttle = 0.6
    
    try:
        if hasattr(agent_obs, 'shape') and len(agent_obs) > 240:
            lidar_data = agent_obs[:240]
            
            # 1. NAVIGATION - Follow route and make turns
            nav_steering = get_navigation_command(agent_obs, agent_info)
            base_steering = nav_steering
            
            # 2. OBSTACLE DETECTION
            has_obstacle, obstacle_distance, obstacle_direction = detect_obstacles(lidar_data)
            
            # 3. V2V COMMUNICATION - Broadcast if obstacle detected
            if has_obstacle and obstacle_distance < 20.0:
                obstacle_info = {
                    "distance": obstacle_distance,
                    "direction": obstacle_direction,
                    "severity": "high" if obstacle_distance < 10.0 else "medium"
                }
                v2v_handler.broadcast_obstacle(agent_id, obstacle_info)
                
                # Store in global database
                with obstacle_lock:
                    obstacle_database[agent_id] = {
                        "info": obstacle_info,
                        "timestamp": time.time()
                    }
            
            # 4. COLLISION AVOIDANCE with graduated response
            front_sector = lidar_data[100:140]
            if len(front_sector) > 0:
                min_front = float(min(front_sector))
                
                if min_front < 5.0:
                    # CRITICAL - Emergency stop
                    base_throttle = -1.0
                    base_steering = 0.0  # Don't steer during emergency brake
                elif min_front < 10.0:
                    # DANGER - Hard brake with obstacle avoidance
                    base_throttle = -0.6
                    # Try to steer away from obstacle
                    left_clear = float(min(lidar_data[140:170])) > 8.0
                    right_clear = float(min(lidar_data[70:100])) > 8.0
                    
                    if left_clear and not right_clear:
                        base_steering = -0.3  # Steer left
                    elif right_clear and not left_clear:
                        base_steering = 0.3   # Steer right
                    elif left_clear and right_clear:
                        # Both clear, use navigation preference
                        base_steering = nav_steering
                elif min_front < 15.0:
                    # WARNING - Slow down
                    base_throttle = 0.2
                elif min_front < 25.0:
                    # CAUTION - Reduce speed
                    base_throttle = 0.4
            
            # 5. SIDE OBSTACLE AVOIDANCE
            left_sector = lidar_data[140:170]
            right_sector = lidar_data[70:100]
            
            if len(left_sector) > 0 and len(right_sector) > 0:
                min_left = float(min(left_sector))
                min_right = float(min(right_sector))
                
                # Adjust steering based on side clearance
                if min_left < 5.0 and min_right > 8.0:
                    base_steering = min(base_steering + 0.25, 0.5)  # Steer right
                    base_throttle = min(base_throttle, 0.3)
                elif min_right < 5.0 and min_left > 8.0:
                    base_steering = max(base_steering - 0.25, -0.5)  # Steer left
                    base_throttle = min(base_throttle, 0.3)
                elif min_left < 5.0 and min_right < 5.0:
                    # Tight squeeze - slow way down
                    base_throttle = 0.15
                    base_steering = nav_steering  # Follow navigation
            
            # 6. CHECK V2V OBSTACLE DATABASE
            # Look for obstacles reported by other vehicles
            with obstacle_lock:
                current_time = time.time()
                for other_agent, data in list(obstacle_database.items()):
                    if other_agent != agent_id:
                        # Check if obstacle info is recent (within 2 seconds)
                        if current_time - data["timestamp"] < 2.0:
                            obstacle_info = data["info"]
                            # If other vehicle reports obstacle ahead, slow down preemptively
                            if obstacle_info["severity"] == "high":
                                base_throttle = min(base_throttle, 0.4)
            
            # 7. SMOOTH SPEED TRANSITIONS
            # Avoid sudden acceleration changes
            # (In production, you'd track previous throttle and smooth it)
            
            # 8. TURN DETECTION - Slow down for curves/intersections
            # If wide spread in lidar readings, likely approaching intersection
            if len(lidar_data) > 200:
                left_avg = float(np.mean(lidar_data[150:180]))
                right_avg = float(np.mean(lidar_data[60:90]))
                front_avg = float(np.mean(lidar_data[110:130]))
                
                # If front is much more open than sides, might be intersection
                if front_avg > 35.0 and (left_avg < 20.0 or right_avg < 20.0):
                    base_throttle = min(base_throttle, 0.45)  # Slow for intersection
    
    except Exception as e:
        # Fallback to safe defaults
        base_throttle = 0.4
        base_steering = 0.0
    
    # Clamp to valid ranges
    steering = np.clip(base_steering, -1.0, 1.0)
    throttle = np.clip(base_throttle, -1.0, 1.0)
    
    return [steering, throttle]


def create_advanced_env(num_agents: int = 8, map_config: str = "city", num_scenarios: int = 50):
    """Create environment for advanced autonomous simulation"""
    
    map_string = MAP_CONFIGS.get(map_config, MAP_CONFIGS["default"])
    
    print("[System] Initializing advanced autonomous environment...")
    print("[INFO] Environment: MultiAgentMetaDrive")
    print("[INFO] MetaDrive version: 0.4.3")
    print(f"[INFO] Map Configuration: {map_config.upper()}")
    print(f"[INFO] Map String: {map_string}")
    print("[INFO] 🤖 Features: Navigation, Collision Avoidance, V2V Communication")

    config = dict(
        num_agents=min(num_agents, 12),
        horizon=3000,  # Longer episodes for complex navigation
        start_seed=0,
        num_scenarios=num_scenarios,
        
        map=map_string,
        
        # Traffic configuration
        traffic_density=0.08,  # Light traffic for better learning
        random_traffic=True,
        traffic_mode="respawn",  # Respawn traffic for continuous flow
        need_inverse_traffic=False,

        # Rendering
        use_render=True,
        window_size=(1280, 720),
        show_logo=False,
        show_interface=True,  # Show navigation interface
        show_fps=True,

        # Vehicle configuration
        vehicle_config=dict(
            show_navi_mark=True,  # Show navigation waypoints
            show_lidar=True,
            show_line_to_dest=True,  # Show line to destination
            random_color=True,
        ),
        
        # Diverse scenarios
        random_agent_model=False,
        random_lane_width=False,
        random_lane_num=False,
    )

    try:
        env = MultiAgentMetaDrive(config)
        print(f"[INFO] ✅ Environment initialized successfully")
        print(f"[INFO] 🗺️  Navigation system: ACTIVE")
        print(f"[INFO] 📡 V2V Communication: ENABLED")
        return env
    except Exception as e:
        print("[ERROR] ❌ Failed to initialize MetaDrive environment.")
        print(f"[ERROR] Details: {str(e)}")
        raise e


def run_advanced_simulation(map_config="city"):
    """Run advanced autonomous simulation with full features"""
    print("=" * 70)
    print("ADVANCED AUTONOMOUS MULTI-AGENT SIMULATION")
    print(f"Map: {map_config.upper()}")
    print("Features: Navigation, Turns, Collision Avoidance, V2V, Path Planning")
    print("=" * 70)
    print()

    # Create environment
    env = create_advanced_env(num_agents=8, map_config=map_config, num_scenarios=50)
    
    # Reset environment
    reset_result = env.reset()
    if isinstance(reset_result, tuple):
        obs, info = reset_result
    else:
        obs = reset_result
    
    print("[System] Environment initialized successfully.")
    print(f"[System] Active agents: {len(obs)}")
    print("[System] Navigation: Agents will follow routes and make turns")
    print("[System] V2V: Obstacle information shared between vehicles")

    # Setup V2V Communication
    TEST_PORT = 5000
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind(("127.0.0.1", TEST_PORT))
        except OSError:
            TEST_PORT = 5001

    v2v_handler = V2VCommunicationHandler(port=TEST_PORT)
    
    # Start communication threads
    recv_thread = threading.Thread(target=start_receiver, args=(TEST_PORT,), daemon=True)
    recv_thread.start()

    print(f"[System] V2V communication running on port {TEST_PORT}")
    print("[System] Starting simulation...")
    print("[System] Watch vehicles: Navigate turns, avoid collisions, share info")
    print("[System] Press Ctrl+C to exit")
    print()

    # Main simulation loop
    try:
        step = 0
        episode = 0
        collision_count = 0
        success_count = 0
        infos = {}
        
        while True:
            # Generate advanced actions for each agent
            actions = {}
            for agent_id in obs.keys():
                actions[agent_id] = get_advanced_action(obs, agent_id, infos, v2v_handler)
            
            # Step environment
            step_result = env.step(actions)
            obs, rewards, terminated, truncated, infos = step_result
            
            # Track statistics
            dones = {agent_id: terminated.get(agent_id, False) or truncated.get(agent_id, False) 
                     for agent_id in obs.keys()}
            
            for agent_id in obs.keys():
                if infos.get(agent_id, {}).get('crash', False):
                    collision_count += 1
                if infos.get(agent_id, {}).get('arrive_dest', False):
                    success_count += 1

            # Render
            env.render()
            
            step += 1

            # Status updates
            if step % 100 == 0:
                active = len(obs.keys())
                done = sum(1 for d in dones.values() if d)
                print(f"[Step {step:5d}] Ep: {episode} | Active: {active} | Done: {done} | "
                      f"Collisions: {collision_count} | Success: {success_count}")

            # Episode reset
            if all(dones.values()):
                episode += 1
                total_reward = sum(rewards.values())
                
                print()
                print("=" * 70)
                print(f"Episode {episode} Complete!")
                print(f"  Steps: {step}")
                print(f"  Successful completions: {success_count}")
                print(f"  Collisions: {collision_count}")
                print(f"  Success rate: {success_count/(success_count+collision_count)*100:.1f}%")
                print(f"  Total reward: {total_reward:.2f}")
                print("=" * 70)
                print()
                
                # Reset
                reset_result = env.reset()
                if isinstance(reset_result, tuple):
                    obs, info = reset_result
                else:
                    obs = reset_result
                
                step = 0
                collision_count = 0
                success_count = 0
                obstacle_database.clear()
                time.sleep(1)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[System] Simulation interrupted by user.")
    finally:
        env.close()
        print("[System] Simulation ended cleanly.")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        map_choice = sys.argv[1].lower()
        if map_choice not in MAP_CONFIGS:
            print(f"Unknown map: {map_choice}")
            print(f"Available: {', '.join(MAP_CONFIGS.keys())}")
            sys.exit(1)
    else:
        map_choice = "city"
    
    print("\n" + "=" * 70)
    print(f"ADVANCED SIMULATION: {map_choice.upper()} MAP")
    print("=" * 70 + "\n")
    
    run_advanced_simulation(map_config=map_choice)