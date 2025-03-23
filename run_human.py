import time

import numpy as np
import pybullet as p

from agent.agent_pybullet import create_snake_pybullet
from agent.food_pybullet import Food
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

    sim_check = True

    # Параметры волны (настраиваются под вашу симуляцию)
    wave_amplitude = 1.0  # амплитуда бокового смещения (может интерпретироваться как величина боковой силы)
    wave_frequency = np.pi / 4  # частота волны
    phase_offset = np.pi / 2  # фазовый сдвиг между сегментами
    steer_offset = 0.0  # дополнительный угол поворота
    base_forward_force = 5  # базовая сила, направленная вперед
    start_time = time.time()

    while sim_check:
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        time.sleep(1.0 / 240.0)
        t = time.time() - start_time

        if p.B3G_LEFT_ARROW in keys:
            pass

        if p.B3G_RIGHT_ARROW in keys:
            pass

        if p.B3G_UP_ARROW in keys:
            # Получаем положение и ориентацию головы для определения направления движения
            head_pos, head_orn = p.getBasePositionAndOrientation(segments[0])
            yaw = p.getEulerFromQuaternion(head_orn)[2]

            # Определяем векторы: "вперёд" и "боковой" (перпендикулярный направлению движения)
            forward_vec = -np.array([np.cos(yaw), np.sin(yaw), 0])
            lateral_vec = np.array([-np.sin(yaw), np.cos(yaw), 0])

            # Применяем силу к каждому сегменту со смещением по синусоидальной зависимости от индекса
            for idx, segment in enumerate(segments):
                # Вычисляем боковую составляющую для текущего сегмента
                lateral_force_component = wave_amplitude * np.sin(
                    2 * np.pi * wave_frequency * t - idx * phase_offset + steer_offset)
                # Итоговый вектор силы: базовая сила вперед плюс боковая составляющая
                force = forward_vec * base_forward_force + lateral_vec * lateral_force_component
                p.applyExternalForce(segment, -1, force, [0, 0, 0], p.WORLD_FRAME)

        if p.B3G_DOWN_ARROW in keys:
            pass

        if ord('q') in keys:
            sim_check = False

        if food.check_collision(segments[0]):
            food.spawn_food()

if __name__ == "__main__":
    snake_human()
