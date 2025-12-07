"""
Path planning and navigation logic
"""
import numpy as np
import time
import config


class PathPlanner:
    def __init__(self):
        self.current_heading = 0.0  # degrees
        self.target_heading = 0.0
        
        self.state = "EXPLORING"  # EXPLORING, AVOIDING, STOPPED
        self.last_turn_time = time.time()
        self.exploration_direction = 1  # 1 = right, -1 = left
        
        # Simple occupancy tracking
        self.visited_headings = set()
        
    def plan(self, distance, obstacles, orientation):
        """
        Main planning logic
        
        Args:
            distance: Ultrasound distance (cm)
            obstacles: List of detected obstacles from camera
            orientation: Dict with pitch, roll, yaw
            
        Returns:
            Dict with 'magnitude', 'angle', 'servo_angle'
        """
        self.current_heading = orientation['yaw']
        
        # Check if stuck (not level)
        if abs(orientation['pitch']) > 30 or abs(orientation['roll']) > 30:
            return self._stop_command("Robot tilted")
            
        # Priority 1: Emergency stop for very close obstacles
        if distance < config.SAFE_DISTANCE:
            return self._avoid_obstacle(distance, obstacles)
            
        # Priority 2: Avoid detected objects in path
        if obstacles:
            return self._avoid_detected_obstacles(obstacles, distance)
            
        # Priority 3: Slow down if approaching obstacle
        if distance < config.SLOW_DISTANCE:
            return self._slow_approach(distance)
            
        # Priority 4: Continue exploration
        return self._explore()
        
    def _avoid_obstacle(self, distance, obstacles):
        """Emergency avoidance"""
        self.state = "AVOIDING"
        
        # Decide turn direction based on obstacles or exploration direction
        if obstacles:
            # Turn away from most obstacles
            avg_angle = np.mean([obs['angle'] for obs in obstacles])
            turn_direction = -1 if avg_angle > 0 else 1
        else:
            turn_direction = self.exploration_direction
            
        # Turn in place
        return {
            'magnitude': 0.0,
            'angle': 90 * turn_direction,  # Turn left or right
            'servo_angle': config.SERVO_CENTER,
            'state': 'AVOIDING',
            'reason': f'Obstacle at {distance:.1f}cm'
        }
        
    def _avoid_detected_obstacles(self, obstacles, distance):
        """Avoid objects detected by camera"""
        self.state = "AVOIDING"
        
        # Find the largest/closest obstacle
        largest_obs = max(obstacles, key=lambda x: x['size'])
        
        # Turn away from it
        if largest_obs['angle'] > 5:
            turn_angle = -30  # Object on right, turn left
        elif largest_obs['angle'] < -5:
            turn_angle = 30   # Object on left, turn right
        else:
            # Object dead center - turn based on exploration direction
            turn_angle = 45 * self.exploration_direction
            
        return {
            'magnitude': config.MIN_SPEED,
            'angle': turn_angle,
            'servo_angle': config.SERVO_CENTER + turn_angle / 2,
            'state': 'AVOIDING',
            'reason': f'Detected {largest_obs["class"]}'
        }
        
    def _slow_approach(self, distance):
        """Slow down when approaching obstacle"""
        # Linear speed reduction
        speed_factor = (distance - config.SAFE_DISTANCE) / (config.SLOW_DISTANCE - config.SAFE_DISTANCE)
        magnitude = config.MIN_SPEED + (config.MAX_SPEED - config.MIN_SPEED) * speed_factor
        magnitude = np.clip(magnitude, config.MIN_SPEED, config.MAX_SPEED)
        
        return {
            'magnitude': magnitude,
            'angle': 0,  # Straight
            'servo_angle': config.SERVO_CENTER,
            'state': 'CAUTIOUS',
            'reason': f'Approaching ({distance:.1f}cm)'
        }
        
    def _explore(self):
        """Free exploration mode"""
        self.state = "EXPLORING"
        current_time = time.time()
        
        # Change direction periodically
        if current_time - self.last_turn_time > 10:
            self.exploration_direction *= -1
            self.last_turn_time = current_time
            
        # Slight turn to cover new area
        turn_angle = 5 * self.exploration_direction
        
        return {
            'magnitude': config.MAX_SPEED,
            'angle': turn_angle,
            'servo_angle': config.SERVO_CENTER + turn_angle,
            'state': 'EXPLORING',
            'reason': 'Free space'
        }
        
    def _stop_command(self, reason="Stop"):
        """Stop the robot"""
        self.state = "STOPPED"
        return {
            'magnitude': 0.0,
            'angle': 0,
            'servo_angle': config.SERVO_CENTER,
            'state': 'STOPPED',
            'reason': reason
        }
