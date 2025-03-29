import time
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import numpy as np
from agent.agent_pybullet import create_snake_pybullet
from models.plane_pybullet import create_walls
from agent.food_pybullet import Food
from utils.setup_pybullet import setup_pybullet


class SnakeEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None):
        super(SnakeEnv, self).__init__()
        self.render_mode = render_mode

        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(low=-10, high=10, shape=(7,), dtype=np.float32)

        setup_pybullet(render_mode=self.render_mode)
        self.food = Food()
        self.reset()

    def reset(self, seed=None, options=None):
        p.resetSimulation()
        self.plane_id = p.loadURDF("plane.urdf")
        p.setGravity(0, 0, -9.8)
        create_walls()

        self.segments, _ = create_snake_pybullet()
        self.snake_head_id = self.segments[0]
        self.food.spawn_food()
        self.start_time = time.time()
        self.last_head_pos = np.array([0, 0, 0])  # Добавляем начальную позицию
        head_pos, _ = p.getBasePositionAndOrientation(self.snake_head_id)
        food_pos, _ = p.getBasePositionAndOrientation(self.food.food_body)
        self.last_distance = np.linalg.norm(np.array(head_pos) - np.array(food_pos))

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
        reward = self._get_reward()
        done = self._is_done()
        return self._get_observation(), reward, done, False, {}

    def _apply_action(self, action):
        force = np.array([0, 0, 0])

        force_multiplier = 40

        if action == 1:  # Вперед
            force = [1, 0, 0]
        elif action == 2:  # Назад
            force = [-1, 0, 0]
        elif action == 3:  # Влево
            force = [0, -1, 0]
        elif action == 4:  # Вправо
            force = [0, 1, 0]

        force = np.array(force) * force_multiplier
        p.applyExternalForce(self.snake_head_id, -1, force, [0, 0, 0], p.WORLD_FRAME)

    def _get_reward(self):
        head_pos, _ = p.getBasePositionAndOrientation(self.snake_head_id)
        food_pos, _ = p.getBasePositionAndOrientation(self.food.food_body)
        current_distance = np.linalg.norm(np.array(head_pos) - np.array(food_pos))

        if self.food.check_collision(self.snake_head_id):
            self.food.spawn_food()
            # Обновляем дистанцию после появления новой еды
            head_pos, _ = p.getBasePositionAndOrientation(self.snake_head_id)
            food_pos, _ = p.getBasePositionAndOrientation(self.food.food_body)
            self.last_distance = np.linalg.norm(np.array(head_pos) - np.array(food_pos))
            return 50.0

        distance_diff = self.last_distance - current_distance

        if distance_diff > 0:
            bonus = distance_diff * 3.0  # Множитель бонуса можно подобрать экспериментально
        else:
            bonus = distance_diff * 0.01  # Штраф за отдаление (будет отрицательным)

        self.last_distance = current_distance

        base_reward = - current_distance * 0.2

        return base_reward + bonus

    def _is_done(self):
        head_pos, _ = p.getBasePositionAndOrientation(self.snake_head_id)
        return abs(head_pos[0]) > 9 or abs(head_pos[1]) > 9
