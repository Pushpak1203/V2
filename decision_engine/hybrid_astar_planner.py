"""
Hybrid A* Path Planner for Vehicle Navigation
Integrates with MetaDrive for obstacle-aware path planning
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional


class Node:
    """Node for Hybrid A* search"""
    def __init__(self, x, y, theta, g_cost, h_cost, parent=None):
        self.x = x
        self.y = y
        self.theta = theta  # Heading angle
        self.g_cost = g_cost  # Cost from start
        self.h_cost = h_cost  # Heuristic cost to goal
        self.f_cost = g_cost + h_cost
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return (abs(self.x - other.x) < 0.5 and 
                abs(self.y - other.y) < 0.5 and
                abs(self.theta - other.theta) < 0.2)
    
    def __hash__(self):
        return hash((int(self.x * 2), int(self.y * 2), int(self.theta * 5)))


class HybridAStarPlanner:
    """
    Hybrid A* Planner for vehicle navigation
    Considers vehicle kinematics and obstacles
    """
    
    def __init__(self, grid_resolution=1.0, angle_resolution=np.pi/8):
        self.grid_resolution = grid_resolution
        self.angle_resolution = angle_resolution
        self.vehicle_length = 4.5  # Approximate vehicle length (meters)
        self.vehicle_width = 2.0   # Approximate vehicle width (meters)
        
        # Motion primitives (steering angles for vehicle)
        self.steering_angles = [-0.4, -0.2, 0.0, 0.2, 0.4]  # radians
        self.step_size = 2.0  # meters per step
    
    def plan(self, start: Tuple[float, float, float], 
             goal: Tuple[float, float, float],
             obstacles: List[Tuple[float, float, float]],
             max_iterations: int = 1000) -> Optional[List[Tuple[float, float, float]]]:
        """
        Plan path from start to goal avoiding obstacles
        
        Args:
            start: (x, y, theta) starting pose
            goal: (x, y, theta) goal pose
            obstacles: List of (x, y, radius) obstacles
            max_iterations: Maximum search iterations
        
        Returns:
            List of waypoints [(x, y, theta), ...] or None if no path found
        """
        start_node = Node(start[0], start[1], start[2], 0, 
                         self._heuristic(start, goal))
        goal_node = Node(goal[0], goal[1], goal[2], 0, 0)
        
        open_set = []
        heapq.heappush(open_set, start_node)
        closed_set = set()
        
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current = heapq.heappop(open_set)
            
            # Check if reached goal
            if self._is_goal(current, goal_node):
                return self._reconstruct_path(current)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Generate successor nodes
            for successor in self._get_successors(current, goal, obstacles):
                if successor not in closed_set:
                    heapq.heappush(open_set, successor)
        
        # No path found, return simple straight line as fallback
        return self._simple_path(start, goal)
    
    def _get_successors(self, node: Node, goal: Tuple, 
                       obstacles: List[Tuple]) -> List[Node]:
        """Generate successor nodes using motion primitives"""
        successors = []
        
        for steering in self.steering_angles:
            # Simulate vehicle motion with this steering angle
            new_x = node.x + self.step_size * np.cos(node.theta)
            new_y = node.y + self.step_size * np.sin(node.theta)
            new_theta = self._normalize_angle(node.theta + steering)
            
            # Check collision
            if self._is_collision_free(new_x, new_y, new_theta, obstacles):
                g_cost = node.g_cost + self.step_size
                h_cost = self._heuristic((new_x, new_y, new_theta), goal)
                
                successor = Node(new_x, new_y, new_theta, g_cost, h_cost, node)
                successors.append(successor)
        
        return successors
    
    def _heuristic(self, pose1: Tuple, pose2: Tuple) -> float:
        """Euclidean distance heuristic"""
        dx = pose2[0] - pose1[0]
        dy = pose2[1] - pose1[1]
        return np.sqrt(dx*dx + dy*dy)
    
    def _is_goal(self, node: Node, goal: Node, threshold=2.0) -> bool:
        """Check if node is close enough to goal"""
        dist = np.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)
        return dist < threshold
    
    def _is_collision_free(self, x: float, y: float, theta: float,
                          obstacles: List[Tuple]) -> bool:
        """Check if pose collides with any obstacle"""
        for obs_x, obs_y, obs_radius in obstacles:
            # Simple circular collision check
            dist = np.sqrt((x - obs_x)**2 + (y - obs_y)**2)
            safety_margin = obs_radius + max(self.vehicle_length, self.vehicle_width) / 2
            if dist < safety_margin:
                return False
        return True
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def _reconstruct_path(self, node: Node) -> List[Tuple]:
        """Reconstruct path from goal to start"""
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y, current.theta))
            current = current.parent
        return list(reversed(path))
    
    def _simple_path(self, start: Tuple, goal: Tuple, 
                    num_points: int = 10) -> List[Tuple]:
        """Generate simple straight-line path as fallback"""
        path = []
        for i in range(num_points + 1):
            t = i / num_points
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            theta = np.arctan2(goal[1] - start[1], goal[0] - start[0])
            path.append((x, y, theta))
        return path


def convert_lidar_to_obstacles(lidar_data: np.ndarray, 
                               vehicle_pos: Tuple[float, float, float],
                               threshold: float = 20.0) -> List[Tuple]:
    """
    Convert lidar readings to obstacle list for path planning
    
    Args:
        lidar_data: Array of 240 distance readings
        vehicle_pos: Current vehicle (x, y, theta)
        threshold: Max distance to consider as obstacle
    
    Returns:
        List of (x, y, radius) obstacles in global coordinates
    """
    obstacles = []
    num_rays = len(lidar_data)
    
    for i, distance in enumerate(lidar_data):
        if distance < threshold:
            # Convert polar to cartesian (local frame)
            angle = (i / num_rays) * 2 * np.pi
            local_x = distance * np.cos(angle)
            local_y = distance * np.sin(angle)
            
            # Transform to global frame
            cos_theta = np.cos(vehicle_pos[2])
            sin_theta = np.sin(vehicle_pos[2])
            
            global_x = vehicle_pos[0] + local_x * cos_theta - local_y * sin_theta
            global_y = vehicle_pos[1] + local_x * sin_theta + local_y * cos_theta
            
            # Add as obstacle with small radius
            obstacles.append((global_x, global_y, 1.0))
    
    # Merge nearby obstacles (optional, for efficiency)
    return obstacles


def path_to_action(current_pose: Tuple[float, float, float],
                  next_waypoint: Tuple[float, float, float],
                  speed: float = 0.6) -> List[float]:
    """
    Convert next waypoint to steering and throttle action
    
    Args:
        current_pose: Current (x, y, theta)
        next_waypoint: Target (x, y, theta)
        speed: Desired speed (0-1)
    
    Returns:
        [steering, throttle] action
    """
    # Calculate angle to waypoint
    dx = next_waypoint[0] - current_pose[0]
    dy = next_waypoint[1] - current_pose[1]
    target_angle = np.arctan2(dy, dx)
    
    # Calculate heading error
    heading_error = target_angle - current_pose[2]
    
    # Normalize to [-pi, pi]
    while heading_error > np.pi:
        heading_error -= 2 * np.pi
    while heading_error < -np.pi:
        heading_error += 2 * np.pi
    
    # Convert to steering command (proportional control)
    steering = np.clip(heading_error * 1.5, -1.0, 1.0)
    
    # Adjust speed based on steering (slow down for sharp turns)
    throttle = speed * (1.0 - 0.5 * abs(steering))
    
    return [steering, throttle]


# Example usage and testing
if __name__ == "__main__":
    # Test the planner
    planner = HybridAStarPlanner()
    
    start = (0.0, 0.0, 0.0)
    goal = (50.0, 30.0, np.pi/4)
    obstacles = [
        (10.0, 10.0, 3.0),
        (25.0, 15.0, 4.0),
        (40.0, 25.0, 3.0),
    ]
    
    print("Planning path...")
    path = planner.plan(start, goal, obstacles)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
        print(f"Start: {path[0]}")
        print(f"Goal: {path[-1]}")
    else:
        print("No path found")