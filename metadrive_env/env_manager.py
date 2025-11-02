from metadrive.envs.marl_envs import MultiAgentMetaDrive

def create_env(num_agents: int = 10, map_name: str = "X", num_scenarios: int = 50, render_mode: str = "onscreen"):
    print("[System] Initializing simulation environment...")
    print("[INFO] Environment: MultiAgentMetaDrive")
    print("[INFO] MetaDrive version: 0.4.3")

    config = dict(
        num_agents=min(num_agents, 15),
        horizon=2000,
        start_seed=0,
        num_scenarios=num_scenarios,

        # ✅ Fixed map configuration
        map=map_name,
        map_config=dict(
            type=map_name,
            lane_width=3.5,
            lane_num=2,
            exit_length=20.0,
            start_position=[(0, 0)],
        ),

        traffic_density=0.15,
        random_traffic=True,
        traffic_mode="hybrid",
        need_inverse_traffic=True,

        use_render=(render_mode == "onscreen"),
        window_size=(1280, 720),
        show_logo=False,
        show_interface=False,
        show_fps=True,

        sensors=dict(
            lidar=("metadrive.component.sensors.lidar.Lidar", {}),
            lane_line_detector=("metadrive.component.sensors.lane_line_detector.LaneLineDetector", {}),
            main_camera=("metadrive.component.sensors.rgb_camera.RGBCamera", dict(width=1280, height=720)),
            dashboard=("metadrive.component.sensors.dashboard.Dashboard", {}),
        ),

        vehicle_config=dict(
            show_navi_mark=True,
            show_lidar=True,
            random_color=True,
        ),
    )

    try:
        env = MultiAgentMetaDrive(config)
        print(f"[INFO] ✅ Environment initialized successfully with map '{map_name}'")
        return env
    except Exception as e:
        print("[ERROR] ❌ Failed to initialize MetaDrive environment.")
        raise e
