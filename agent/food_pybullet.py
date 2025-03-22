import pybullet as p
import random
import numpy as np

class Food:
    def __init__(self):
        self.food_body = None

    def spawn_food(self):
        if self.food_body is not None:
            try:
                p.removeBody(self.food_body)
            except:
                print("ошибка удаления еды")
        x, y = random.uniform(-8, 8), random.uniform(-8, 8)
        z = 1
        self.food_body = p.createMultiBody(baseMass=1,
                                           baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE,
                                                                                          radius=0.15),
                                           baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE,
                                                                                    radius=0.15,
                                                                                    rgbaColor=[1, 0, 0, 1]),
                                           basePosition=[x, y, z])

    def check_collision(self, snake_head_id):
        head_pos, _ = p.getBasePositionAndOrientation(snake_head_id)
        food_pos, _ = p.getBasePositionAndOrientation(self.food_body)
        return np.linalg.norm(np.array(head_pos) - np.array(food_pos)) < 0.3