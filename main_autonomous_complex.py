"""
Main simulation with complex procedurally generated maps
Realistic road networks with intersections, curves, and roundabouts
"""

from metadrive.envs.marl_envs import MultiAgentMetaDrive
from communication.broadcaster import start_broadcaster
from communication.receiver import start_receiver
import threading
import time
import socket
import sys


# Available map configurations
MAP_CONFIGS = {
    "city": "XCTOXCTOX",  # City-like: Intersections, Curves, T-junctions, Roundabouts
    "highway": "SSSCSSSCSSS",  # Highway: Long straights with curves
    "complex": "XCTOCSXTOS",  # Complex mixed roads
    "roundabouts": "OOOOO",  # Multiple roundabouts
    "urban": "XTXTXTXT",  # Urban grid with intersections and T-junctions
    "suburban": "SCSCSCSC",  # Suburban: Straights and curves
    "default": "XCTOX",  # Balanced default
}


def create_complex_env(num_agents: int = 10, map_config: str = "city", num_scenarios: int = 50):
    """Create environment with complex procedural maps"""
    
    map_string = MAP_CONFIGS.get(map_config, MAP_CONFIGS["default"])
    
    print("[System] Initializing complex simulation environment...")
    print("[INFO] Environment: MultiAgentMetaDrive")
    print("[INFO] MetaDrive version: 0.4.3")
    print(f"[INFO] Map Configuration: {map_config.upper()}")
    print(f"[INFO] Map String: {map_string}")
    print("[INFO] 🤖 All agents: Autonomous (Cruise Control)")

    config = dict(
        num_agents=min(num_agents, 15),
        horizon=2000,
        start_seed=0,
        num_scenarios=num_scenarios,
        
        # Procedural map generation
        # Letters: S=Straight, C=Curve, X=4-way intersection, T=T-junction, O=Roundabout
        map=map_string,
        
        # Traffic configuration
        traffic_density=0.2,  # More traffic vehicles
        random_traffic=True,
        traffic_mode="hybrid",
        need_inverse_traffic=True,

        # Rendering
        use_render=True,
        window_size=(1280, 720),
        show_logo=False,
        show_interface=False,
        show_fps=True,

        # Vehicle configuration
        vehicle_config=dict(
            show_navi_mark=True,
            show_lidar=True,
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
        print(f"[INFO] 🗺️  Map: {map_config} - {decode_map(map_string)}")
        return env
    except Exception as e:
        print("[ERROR] ❌ Failed to initialize MetaDrive environment.")
        print(f"[ERROR] Details: {str(e)}")
        raise e


def decode_map(map_string: str) -> str:
    """Decode map string to readable description"""
    legend = {
        'S': 'Straight',
        'C': 'Curve',
        'X': '4-Way Intersection',
        'T': 'T-Junction',
        'O': 'Roundabout'
    }
    parts = [legend.get(char, char) for char in map_string]
    return " → ".join(parts)


def get_safe_action(obs, agent_id):
    """
    Generate safe action based on sensor data to avoid collisions.
    Uses lidar and side detector to maintain safe distances.
    """
    agent_obs = obs[agent_id]
    
    # Default action: moderate acceleration, no steering
    steering = 0.0
    throttle = 0.7
    
    # Get lidar data (240 points covering 360 degrees)
    lidar = agent_obs.get('lidar', {}).get('cloud_points', [])
    
    if len(lidar) > 0:
        # Check front sector (roughly -30 to +30 degrees)
        # Lidar points are arranged in a circle, front is around indices 90-150
        front_sector = lidar[90:150]
        
        if len(front_sector) > 0:
            # Find minimum distance in front
            min_front_distance = min(front_sector)
            
            # Safety thresholds
            DANGER_DISTANCE = 10.0  # Very close - brake hard
            WARNING_DISTANCE = 20.0  # Getting close - slow down
            SAFE_DISTANCE = 30.0  # Comfortable distance
            
            if min_front_distance < DANGER_DISTANCE:
                # Emergency brake
                throttle = -0.5
            elif min_front_distance < WARNING_DISTANCE:
                # Slow down significantly
                throttle = 0.2
            elif min_front_distance < SAFE_DISTANCE:
                # Gentle deceleration
                throttle = 0.4
            else:
                # Safe to proceed
                throttle = 0.7
        
        # Check left and right for lane changes (if needed)
        left_sector = lidar[150:180]
        right_sector = lidar[60:90]
        
        if len(left_sector) > 0 and len(right_sector) > 0:
            min_left = min(left_sector)
            min_right = min(right_sector)
            
            # Slight steering adjustments to avoid very close obstacles
            if min_left < 5.0:
                steering = 0.1  # Steer slightly right
            elif min_right < 5.0:
                steering = -0.1  # Steer slightly left
    
    # Get side detector data for additional safety
    side_detector = agent_obs.get('side_detector', [])
    if isinstance(side_detector, list) and len(side_detector) >= 4:
        # side_detector: [left_front, left_rear, right_rear, right_front]
        left_front, left_rear, right_rear, right_front = side_detector
        
        # If vehicles very close on sides, be extra careful
        if left_front < 5.0 or right_front < 5.0:
            throttle = min(throttle, 0.3)  # Slow down if cars adjacent
    
    return [steering, throttle]


def run_complex_simulation(map_config="city"):
    """Run autonomous multi-agent simulation with complex maps and collision avoidance."""
    print("=" * 70)
    print("AUTONOMOUS MULTI-AGENT SIMULATION")
    print(f"Complex Road Network: {map_config.upper()}")
    print("With Collision Avoidance System")
    print("=" * 70)
    print()

    # --- Step 1: Setup Environment ---
    env = create_complex_env(num_agents=15, map_config=map_config, num_scenarios=50)
    
    # Reset environment
    reset_result = env.reset()
    if isinstance(reset_result, tuple):
        obs, info = reset_result
    else:
        obs = reset_result
    
    print("[System] Simulation environment initialized successfully.")
    print(f"[System] Active agents: {len(obs)}")
    print("[System] Collision avoidance: ENABLED")

    # --- Step 2: Safe Socket Initialization ---
    TEST_PORT = 5000
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.bind(("127.0.0.1", TEST_PORT))
        except OSError:
            print(f"[Warning] Port {TEST_PORT} already in use, switching to 5001.")
            TEST_PORT = 5001

    # --- Step 3: Start Communication Threads ---
    recv_thread = threading.Thread(target=start_receiver, args=(TEST_PORT,), daemon=True)
    send_thread = threading.Thread(target=start_broadcaster, args=("Autonomous_Agent", TEST_PORT), daemon=True)
    recv_thread.start()
    send_thread.start()

    print(f"[System] Communication threads running on port {TEST_PORT}")
    print("[System] Starting autonomous simulation with collision avoidance...")
    print("[System] Press Ctrl+C to exit")
    print()

    # --- Step 4: Main Simulation Loop ---
    try:
        step = 0
        episode = 0
        collision_count = 0
        
        while True:
            # Generate safe actions for each agent using sensor data
            actions = {}
            for agent_id in obs.keys():
                actions[agent_id] = get_safe_action(obs, agent_id)
            
            # Step the environment
            step_result = env.step(actions)
            obs, rewards, terminated, truncated, infos = step_result
            
            # Combine terminated and truncated
            dones = {agent_id: terminated.get(agent_id, False) or truncated.get(agent_id, False) 
                     for agent_id in obs.keys()}
            
            # Count collisions
            for agent_id in obs.keys():
                if infos.get(agent_id, {}).get('crash', False):
                    collision_count += 1

            # Render the simulation
            env.render()
            
            step += 1

            # Print status every 100 steps
            if step % 100 == 0:
                active_agents = len(obs.keys())
                done_agents = sum(1 for done in dones.values() if done)
                print(f"[Step {step:5d}] Episode: {episode} | Active: {active_agents} | Done: {done_agents} | Collisions: {collision_count}")

            # Reset when all agents are done
            if all(dones.values()):
                episode += 1
                
                # Calculate episode statistics
                total_reward = sum(rewards.values())
                successes = sum(1 for agent_id in obs.keys() 
                               if infos.get(agent_id, {}).get('arrive_dest', False))
                
                print()
                print("=" * 70)
                print(f"Episode {episode} Complete!")
                print(f"  Steps: {step}")
                print(f"  Successful agents: {successes}/{len(obs)}")
                print(f"  Total collisions: {collision_count}")
                print(f"  Total reward: {total_reward:.2f}")
                print("=" * 70)
                print()
                
                # Reset environment
                reset_result = env.reset()
                if isinstance(reset_result, tuple):
                    obs, info = reset_result
                else:
                    obs = reset_result
                
                step = 0
                collision_count = 0
                time.sleep(1)  # Brief pause between episodes

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[System] Simulation interrupted by user.")
    finally:
        env.close()
        print("[System] Simulation ended cleanly.")


def print_map_options():
    """Print available map configurations"""
    print("\n" + "=" * 70)
    print("AVAILABLE MAP CONFIGURATIONS")
    print("=" * 70)
    for name, map_string in MAP_CONFIGS.items():
        print(f"\n{name.upper()}:")
        print(f"  Map String: {map_string}")
        print(f"  Description: {decode_map(map_string)}")
    print("\n" + "=" * 70)


if __name__ == "__main__":
    # Check for command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == "--help" or sys.argv[1] == "-h":
            print("\nUsage: python main_autonomous_complex.py [map_config]")
            print("\nAvailable map configurations:")
            for name in MAP_CONFIGS.keys():
                print(f"  - {name}")
            print("\nExamples:")
            print("  python main_autonomous_complex.py city")
            print("  python main_autonomous_complex.py highway")
            print("  python main_autonomous_complex.py complex")
            print_map_options()
            sys.exit(0)
        
        map_choice = sys.argv[1].lower()
        if map_choice not in MAP_CONFIGS:
            print(f"[ERROR] Unknown map configuration: {map_choice}")
            print(f"[INFO] Available maps: {', '.join(MAP_CONFIGS.keys())}")
            sys.exit(1)
    else:
        map_choice = "city"  # Default
    
    print("\n" + "=" * 70)
    print(f"Starting simulation with '{map_choice.upper()}' map configuration")
    print("=" * 70 + "\n")
    
    run_complex_simulation(map_config=map_choice)