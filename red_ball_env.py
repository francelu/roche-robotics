import gymnasium as gym
import habitat
from habitat.core.environments import get_env_class
from habitat.config.default import get_config
import numpy as np
from gymnasium.spaces import Box, Discrete
import cv2

class RedBallHabitatEnv(gym.Env):
    def __init__(self, scene_path, max_steps=500):
        super().__init__()
        # Load Habitat config
        self.config = get_config()
        self.config.defrost()
        self.config.TASK.POSSIBLE_ACTIONS = ["MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"]
        self.config.SIMULATOR.SCENE = "data/scene_datasets/simple_room/empty_room.glb"  # Path to your 3D scene (e.g., .glb file)
        self.config.SIMULATOR.AGENT_0.SENSORS = ["RGB_SENSOR"]
        self.config.SIMULATOR.RGB_SENSOR.WIDTH = 128
        self.config.SIMULATOR.RGB_SENSOR.HEIGHT = 128
        self.config.SIMULATOR.ACTION_SPACE_CONFIG = "mobile_robot"
        self.config.TASK.MEASUREMENTS = ["DISTANCE_TO_GOAL", "SUCCESS"]
        self.config.TASK.SUCCESS_DISTANCE = 0.2  # Success if within 0.2m of the ball
        self.config.freeze()

        # Initialize Habitat environment
        self.habitat_env = habitat.Env(config=self.config)
        self.max_steps = max_steps
        self.current_step = 0

        # Define observation and action spaces
        self.observation_space = Box(
            low=0, high=255, shape=(128, 128, 3), dtype=np.uint8
        )  # RGB images
        self.action_space = Discrete(4)  # 0: forward, 1: left, 2: right, 3: stop

        # Red ball position (example: set randomly or fixed in the scene)
        self.ball_position = np.array([1.0, 0.0, 1.0])  # Example coordinates (x, y, z)

    def reset(self, seed=None, options=None):
        self.current_step = 0
        # Reset Habitat environment
        observations = self.habitat_env.reset()
        # Optionally, randomize ball position here
        # self.ball_position = randomize_position()  # Implement as needed
        return observations["rgb"], {}  # Return RGB image and empty info dict

    def step(self, action):
        self.current_step += 1
        # Map Gym action to Habitat action
        habitat_action = ["MOVE_FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"][action]
        observations = self.habitat_env.step(action=habitat_action)

        # Compute reward
        distance_to_ball = self._compute_distance_to_ball()
        reward = -0.01  # Small negative reward per step (encourage efficiency)
        if distance_to_ball < 0.2:  # Success condition
            reward += 10.0
        if self.habitat_env.get_metrics()["success"]:
            reward += 100.0  # Large reward for reaching the ball

        # Check termination
        done = (
            self.habitat_env.get_metrics()["success"]
            or self.current_step >= self.max_steps
        )
        truncated = self.current_step >= self.max_steps

        return observations["rgb"], reward, done, truncated, {}

    def _compute_distance_to_ball(self):
        # Get agent position from Habitat
        agent_position = self.habitat_env.sim.get_agent_state().position
        return np.linalg.norm(agent_position - self.ball_position)

    def render(self):
        # Return RGB image for visualization
        return self.habitat_env.get_observations()["rgb"]

    def close(self):
        self.habitat_env.close()