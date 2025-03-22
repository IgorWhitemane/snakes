import time

import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import numpy as np
from agent.agent_pybullet import create_snake_pybullet
from agent.food_pybullet import Food
from utils.setup_pybullet import setup_pybullet


class SnakeEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super(SnakeEnv, self).__init__()
        self.render_mode = render_mode
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(low=-10, high=10, shape=(7,), dtype=np.float32)
        setup_pybullet(render_mode=self.render_mode)
        self.plane_id = p.loadURDF("plane.urdf")
        self.food = Food()
        self.reset()

    def reset(self, seed=None, options=None):
        p.resetSimulation()
        self.plane_id = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.8)
        self.segments, _ = create_snake_pybullet()
        self.snake_head_id = self.segments[0]
        self.food.spawn_food()
        return self._get_observation(), {}

    def _get_observation(self):
        """Возвращает текущее состояние змейки и еды."""
        head_pos, head_orn = p.getBasePositionAndOrientation(self.snake_head_id)
        food_pos, _ = p.getBasePositionAndOrientation(self.food.food_body)
        head_euler = p.getEulerFromQuaternion(head_orn)

        return np.array([
            head_pos[0], head_pos[1], head_pos[2],
            food_pos[0], food_pos[1], food_pos[2],
            head_euler[2]  # Угол поворота головы
        ], dtype=np.float32)

    def step(self, action):
        self._apply_action(action)
        p.stepSimulation()
        # time.sleep(1. / 240.)
        reward = self._get_reward()
        done = self._is_done()
        return self._get_observation(), reward, done, False, {}

    def _apply_action(self, action):
        head_pos, head_orn = p.getBasePositionAndOrientation(self.snake_head_id)
        yaw = p.getEulerFromQuaternion(head_orn)[2]

        if action == 0:
            pass
        elif action == 1:
            yaw += np.pi
        elif action == 2:
            yaw -= np.pi / 4
        elif action == 3:
            yaw += np.pi / 4

        new_orn = p.getQuaternionFromEuler([0, 0, yaw])
        force = [np.cos(yaw) * 5, np.sin(yaw) * 5, 0]
        p.applyExternalForce(self.snake_head_id, -1, force, [0, 0, 0], p.WORLD_FRAME)

    def _get_reward(self):
        if self.food.check_collision(self.snake_head_id):
            self.food.spawn_food()
            return 50.0
        head_pos, _ = p.getBasePositionAndOrientation(self.snake_head_id)
        food_pos, _ = p.getBasePositionAndOrientation(self.food.food_body)
        return -np.linalg.norm(np.array(head_pos) - np.array(food_pos)) * 0.05

    def _is_done(self):
        head_pos, _ = p.getBasePositionAndOrientation(self.snake_head_id)
        return abs(head_pos[0]) > 9 or abs(head_pos[1]) > 9
