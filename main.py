from metadrive_env.env_manager import create_env
from communication.broadcaster import start_broadcaster
from communication.receiver import start_receiver
import threading
import time
import socket


def run_simulation():
    """Run the MetaDrive multi-agent simulation with communication system."""
    print("[System] Initializing simulation environment...")

    # --- Step 1: Setup Environment (Procedural Map, Safe Defaults) ---
    env = create_env(num_agents=15, map_name="X", num_scenarios=50, render_mode="onscreen")
    
    # ✅ Fixed: env.reset() returns (obs, info) tuple in Gymnasium API
    reset_result = env.reset()
    if isinstance(reset_result, tuple):
        obs, info = reset_result
    else:
        obs = reset_result
    
    print("[System] Simulation environment initialized successfully.")

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
    send_thread = threading.Thread(target=start_broadcaster, args=("Agent_1", TEST_PORT), daemon=True)
    recv_thread.start()
    send_thread.start()

    print("[System] Communication threads running. Starting simulation...")

    # --- Step 4: Main Simulation Loop ---
    try:
        for step in range(1000):
            # Each observation is a dict (agent_id -> obs)
            actions = {agent_id: [0.0, 1.0] for agent_id in obs.keys()}
            
            # ✅ Fixed: env.step() returns (obs, rewards, terminated, truncated, infos)
            step_result = env.step(actions)
            obs, rewards, terminated, truncated, infos = step_result
            
            # Combine terminated and truncated to get dones
            dones = {agent_id: terminated.get(agent_id, False) or truncated.get(agent_id, False) 
                     for agent_id in obs.keys()}

            # Render the simulation
            env.render()

            # Reset when all agents are done
            if all(dones.values()):
                reset_result = env.reset()
                if isinstance(reset_result, tuple):
                    obs, info = reset_result
                else:
                    obs = reset_result

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("[System] Simulation interrupted by user.")
    finally:
        env.close()
        print("[System] Simulation ended cleanly.")


if __name__ == "__main__":
    run_simulation()