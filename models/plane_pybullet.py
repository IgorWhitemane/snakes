import pybullet as p
import numpy as np

def create_walls(arena_size=5, wall_height=10):
    """Создает стены вокруг игрового поля."""
    wall_thickness = 0.1  # Толщина стен

    wall_positions = [
        [arena_size, 0, wall_height / 2],  # Правая стена
        [-arena_size, 0, wall_height / 2],  # Левая стена
        [0, arena_size, wall_height / 2],  # Верхняя стена
        [0, -arena_size, wall_height / 2]  # Нижняя стена
    ]

    wall_orientations = [
        [0, 0, 0, 1],  # Без поворота
        [0, 0, 0, 1],  # Без поворота
        [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)],  # 90 градусов вокруг Z
        [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)]   # 90 градусов вокруг Z
    ]

    walls = []
    for pos, orn in zip(wall_positions, wall_orientations):
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness, arena_size, wall_height])
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness, arena_size, wall_height], rgbaColor=[0.5, 0.5, 0.5, 1])
        wall_id = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision_shape,
                                    baseVisualShapeIndex=visual_shape, basePosition=pos, baseOrientation=orn)
        walls.append(wall_id)
