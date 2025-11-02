"""
Enhanced environment manager with IDM (Intelligent Driver Model) policy
for more realistic autonomous driving behavior
"""

from metadrive.envs.marl_envs import MultiAgentMetaDrive
from metadrive.policy.idm_policy import IDMPolicy

def create_env_with_idm(num_agents: int = 10, map_name: str = "X", num_scenarios: int = 50, 
                        render_mode: str = "onscreen", use_idm: bool = True):
    """
    Create environment with optional IDM policy for realistic autonomous driving.
    
    Args:
        num_agents: Number of agents (max 15)
        map_name: Map layout ("X", "S", "C", "O", "T")
        num_scenarios: Number of different scenarios
        render_mode: "onscreen" or "offscreen"
        use_idm: Use Intelligent Driver Model for autonomous behavior
    """
    print("[System] Initializing simulation environment...")
    print("[INFO] Environment: MultiAgentMetaDrive")
    print("[INFO] MetaDrive version: 0.4.3")
    
    if use_idm:
        print("[INFO] Agent Policy: IDM (Intelligent Driver Model)")
        print("[INFO] Features: Car-following, safe distance, collision avoidance")
    else:
        print("[INFO] Agent Policy: Manual (controlled via actions)")

    config = dict(
        num_agents=min(num_agents, 15),
        horizon=2000,
        start_seed=0,
        num_scenarios=num_scenarios,

        # Map configuration
        map=map_name,
        
        # Traffic configuration
        traffic_density=0.15,
        random_traffic=True,
        traffic_mode="hybrid",
        need_inverse_traffic=True,

        # Rendering
        use_render=(render_mode == "onscreen"),
        window_size=(1280, 720),
        show_logo=False,
        show_interface=False,
        show_fps=True,

        # IDM Policy configuration
        agent_policy=IDMPolicy if use_idm else None,
        
        # Vehicle configuration
        vehicle_config=dict(
            show_navi_mark=True,
            show_lidar=True,
            random_color=True,
            
            # IDM-specific parameters
            spawn_lane_index=None,  # Random spawn
        ),
        
        # Success/Failure conditions
        out_of_road_penalty=5.0,
        crash_vehicle_penalty=5.0,
        crash_object_penalty=5.0,
        success_reward=10.0,
    )

    try:
        env = MultiAgentMetaDrive(config)
        print(f"[INFO] ✅ Environment initialized successfully with map '{map_name}'")
        if use_idm:
            print("[INFO] 🤖 All agents will drive autonomously using IDM policy")
        return env
    except Exception as e:
        print("[ERROR] ❌ Failed to initialize MetaDrive environment.")
        print(f"[ERROR] Details: {str(e)}")
        raise e


def create_env(num_agents: int = 10, map_name: str = "X", num_scenarios: int = 50, 
               render_mode: str = "onscreen"):
    """
    Original create_env function for backward compatibility.
    This version does NOT use IDM policy (manual control mode).
    """
    return create_env_with_idm(
        num_agents=num_agents,
        map_name=map_name,
        num_scenarios=num_scenarios,
        render_mode=render_mode,
        use_idm=False  # Default to manual control for compatibility
    )


# Example usage:
if __name__ == "__main__":
    import time
    
    print("Testing IDM-enabled environment...")
    
    # Create environment with IDM policy
    env = create_env_with_idm(num_agents=5, map_name="X", use_idm=True)
    
    # Reset
    reset_result = env.reset()
    if isinstance(reset_result, tuple):
        obs, info = reset_result
    else:
        obs = reset_result
    
    print(f"[Test] Environment ready with {len(obs)} agents")
    print("[Test] Running 100 steps with IDM policy...")
    
    # Run simulation
    for step in range(100):
        # With IDM policy, pass None as actions
        actions = None
        
        # Step
        step_result = env.step(actions)
        obs, rewards, terminated, truncated, infos = step_result
        
        # Render
        env.render()
        
        if step % 20 == 0:
            print(f"[Test] Step {step}/100")
        
        time.sleep(0.02)
    
    env.close()
    print("[Test] ✅ Test completed successfully!")