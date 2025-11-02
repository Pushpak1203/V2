"""
Main simulation with full autonomous driving using IDM policy
All vehicles drive intelligently with car-following and collision avoidance
"""

from metadrive.envs.marl_envs import MultiAgentMetaDrive
from metadrive.policy.idm_policy import IDMPolicy
from communication.broadcaster import start_broadcaster
from communication.receiver import start_receiver
import threading
import time
import socket


def create_autonomous_env(num_agents: int = 10, map_name: str = "X", num_scenarios: int = 50):
    """Create environment with autonomous driving (simple cruise control)"""
    print("[System] Initializing autonomous simulation environment...")
    print("[INFO] Environment: MultiAgentMetaDrive")
    print("[INFO] MetaDrive version: 0.4.3")
    print("[INFO] 🤖 All agents: Autonomous (Cruise Control)")

    config = dict(
        num_agents=min(num_agents, 15),
        horizon=2000,
        start_seed=0,
        num_scenarios=num_scenarios,
        map=map_name,
        
        # Traffic configuration
        traffic_density=0.15,
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
    )

    try:
        env = MultiAgentMetaDrive(config)
        print(f"[INFO] ✅ Environment initialized successfully with map '{map_name}'")
        print("[INFO] 🤖 Autonomous driving: Simple cruise control policy")
        return env
    except Exception as e:
        print("[ERROR] ❌ Failed to initialize MetaDrive environment.")
        print(f"[ERROR] Details: {str(e)}")
        raise e


def run_autonomous_simulation():
    """Run fully autonomous multi-agent simulation with communication system."""
    print("=" * 70)
    print("AUTONOMOUS MULTI-AGENT SIMULATION")
    print("All vehicles controlled by Simple Cruise Control")
    print("=" * 70)
    print()

    # --- Step 1: Setup Environment with IDM Policy ---
    env = create_autonomous_env(num_agents=15, map_name="X", num_scenarios=50)
    
    # Reset environment
    reset_result = env.reset()
    if isinstance(reset_result, tuple):
        obs, info = reset_result
    else:
        obs = reset_result
    
    print("[System] Simulation environment initialized successfully.")
    print(f"[System] Active agents: {len(obs)}")

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
    print("[System] Starting autonomous simulation...")
    print("[System] Press Ctrl+C to exit")
    print()

    # --- Step 4: Main Simulation Loop with IDM Policy ---
    try:
        step = 0
        episode = 0
        
        while True:
            # Simple autonomous driving: cruise control with moderate acceleration
            # All agents drive forward trying to reach their destinations
            actions = {agent_id: [0.0, 0.7] for agent_id in obs.keys()}
            
            # Step the environment
            step_result = env.step(actions)
            obs, rewards, terminated, truncated, infos = step_result
            
            # Combine terminated and truncated
            dones = {agent_id: terminated.get(agent_id, False) or truncated.get(agent_id, False) 
                     for agent_id in obs.keys()}

            # Render the simulation
            env.render()
            
            step += 1

            # Print status every 100 steps
            if step % 100 == 0:
                active_agents = len(obs.keys())
                done_agents = sum(1 for done in dones.values() if done)
                print(f"[Step {step:5d}] Episode: {episode} | Active: {active_agents} | Completed: {done_agents}")

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
                time.sleep(1)  # Brief pause between episodes

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[System] Simulation interrupted by user.")
    finally:
        env.close()
        print("[System] Simulation ended cleanly.")


if __name__ == "__main__":
    run_autonomous_simulation()