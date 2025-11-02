def plan_path(start, goal):
    """
    Placeholder for Hybrid A* path planner.
    Currently returns a simple straight-line trajectory.
    """
    path = [start]
    while abs(goal[0] - path[-1][0]) > 0.1:
        step = (goal[0] - path[-1][0]) * 0.05
        path.append((path[-1][0] + step, path[-1][1]))
    return path
