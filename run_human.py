import time

import numpy as np
import pybullet as p

from agent.agent_2 import create_snake_pybullet_2
# from agent.agent_pybullet import create_snake_pybullet
from agent.food_pybullet import Food
from utils.setup_pybullet import setup_pybullet

def snake_human():
    setup_pybullet("human")
    plane_id = p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    # Физика (гравитация и трение) плоскости
    p.setGravity(0, 0, -9.8)
    p.changeDynamics(plane_id, -1, lateralFriction=2.0)

    # Создание змеи
    snake_id, joints = create_snake_pybullet_2()

    # Задаем трение для каждого сегмента змеи
    for segment in range(len(joints)):
        p.changeDynamics(snake_id, segment, anisotropicFriction=[0.5, 1.0, 1.0])

    # Управление
    food = Food()
    food.spawn_food()

    sim_check = True
    wave_frequency = 2.0
    wave_amplitude = np.pi / 8
    phase_offset = np.pi / 4

    base_forward_force = 10.0
    steer_offset = 0.0
    start_time = time.time()

    while sim_check:
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        t = time.time() - start_time

        if p.B3G_LEFT_ARROW in keys:
            steer_offset += 0.5
        if p.B3G_RIGHT_ARROW in keys:
            steer_offset -= 0.5
        current_forward_force = base_forward_force

        if p.B3G_UP_ARROW in keys:
            # Управление суставами
            for idx in joints:
                target_angle = wave_amplitude * np.sin(
                    2 * np.pi * wave_frequency * t - idx * phase_offset + steer_offset)
                p.setJointMotorControl2(
                    bodyUniqueId=snake_id,
                    jointIndex=idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_angle,
                    force=20
                )
            # Применяем силу к голове для продвижения вперед
            head_pos, head_orn = p.getBasePositionAndOrientation(snake_id)
            yaw = p.getEulerFromQuaternion(head_orn)[2]
            force = [np.cos(yaw) * current_forward_force, np.sin(yaw) * current_forward_force, 0]
            p.applyExternalForce(snake_id, -1, force, [0, 0, 0], p.WORLD_FRAME)

        if ord('q') in keys:
            sim_check = False

        if food.check_collision(snake_id):
            food.spawn_food()

        time.sleep(1.0/240.0)


if __name__ == "__main__":
    snake_human()
