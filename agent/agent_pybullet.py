import pybullet as p

def create_snake_pybullet(segment_count=10, segment_radius=0.1, start_position=(0, 0, 1)):
    segments = []
    joints = []
    head_radius = segment_radius * 1.1
    x, y, z = start_position

    for i in range(segment_count):
        radius = head_radius if i == 0 else segment_radius
        color = [0.8, 0.7, 0.3, 1] if i == 0 else [0, 0.8, 0, 1]

        segment_position = [x + i * (2 * segment_radius * 1.1), y, z]

        segment_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
        visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
        body_id = p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=segment_id,
                                    baseVisualShapeIndex=visual_id, basePosition=segment_position)
        segments.append(body_id)

        if i > 0:
            parent_position = [segment_radius * 1.1, 0, 0]
            child_position = [-segment_radius * 1.1, 0, 0]
            joint_id = p.createConstraint(
                parentBodyUniqueId=segments[i - 1],
                parentLinkIndex=-1,
                childBodyUniqueId=segments[i],
                childLinkIndex=-1,
                jointType=p.JOINT_POINT2POINT,
                jointAxis=[0, 0, 1],
                parentFramePosition=parent_position,
                childFramePosition=child_position
            )
            joints.append(joint_id)

    return segments, joints

