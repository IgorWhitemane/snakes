import time

import numpy as np
import pybullet as p

from agent.agent_pybullet import create_snake_pybullet
from agent.food_pybullet import Food
from models.plane_pybullet import create_walls
from utils.setup_pybullet import setup_pybullet


def snake_human():
    setup_pybullet("human")
    plane_id = p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Физика (гравитация и трение) плоскости
    p.setGravity(0, 0, -9.8)
    p.changeDynamics(plane_id, -1, lateralFriction=1.0)

    # Создание змеи
    segments, joints = create_snake_pybullet()

    # Управление
    food = Food()
    food.spawn_food()
    create_walls()

    sim_check = True

    # Параметры волны (настраиваются под вашу симуляцию)
    wave_amplitude = 2.0  # амплитуда бокового смещения (может интерпретироваться как величина боковой силы)
    wave_frequency = np.pi / 8  # частота волны
    phase_offset = np.pi / 4  # фазовый сдвиг между сегментами
    steer_offset = 0.0  # дополнительный угол поворота
    start_time = time.time()

    force_multiplier = 50

    while sim_check:
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        time.sleep(1.0 / 240.0)
        force = np.array([0, 0, 0])

        if p.B3G_UP_ARROW in keys:  # Вперед
            force = [1, 0, 0]
        elif p.B3G_DOWN_ARROW in keys:  # Назад
            force = [-1, 0, 0]
        elif p.B3G_LEFT_ARROW in keys:  # Влево
            force = [0, -1, 0]
        elif p.B3G_RIGHT_ARROW in keys:  # Вправо
            force = [0, 1, 0]

        force = np.array(force) * force_multiplier
        p.applyExternalForce(segments[0], -1, force, [0, 0, 0], p.WORLD_FRAME)

        if ord('q') in keys:
            sim_check = False

        if food.check_collision(segments[0]):
            food.spawn_food()

if __name__ == "__main__":
    snake_human()

