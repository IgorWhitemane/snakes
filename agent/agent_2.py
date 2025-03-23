import pybullet as p

def create_snake_pybullet_2(segment_count=3, segment_radius=0.1, start_position=(0, 0, 1)):
    # Создаем голову змейки (базу мульти-тела)
    head_radius = segment_radius * 1.1
    base_collision_shape_index = p.createCollisionShape(p.GEOM_SPHERE, radius=head_radius)
    base_visual_shape_index = p.createVisualShape(p.GEOM_SPHERE, radius=head_radius, rgbaColor=[0.8, 0.7, 0.3, 1])
    base_mass = 0.5
    base_position = start_position
    base_orientation = [0, 0, 0, 1]

    # Для остальных сегментов зададим одинаковые параметры
    link_masses = [0.5] * (segment_count - 1)
    link_collision_shape_indices = [p.createCollisionShape(p.GEOM_SPHERE, radius=segment_radius)
                                    for _ in range(segment_count - 1)]
    link_visual_shape_indices = [p.createVisualShape(p.GEOM_SPHERE, radius=segment_radius,
                                                     rgbaColor=[0, 0.8, 0, 1])
                                 for _ in range(segment_count - 1)]
    link_positions = [[2 * segment_radius * 1.1, 0, 0] for _ in range(segment_count - 1)]
    link_orientations = [[0, 0, 0, 1] for _ in range(segment_count - 1)]
    link_inertial_frame_positions = [[0, 0, 0] for _ in range(segment_count - 1)]
    link_inertial_frame_orientations = [[0, 0, 0, 1] for _ in range(segment_count - 1)]
    link_parent_indices = [i for i in range(segment_count - 1)]
    link_joint_types = [p.JOINT_SPHERICAL] * (segment_count - 1)
    link_joint_axis = [[0, 0, 0] for _ in range(segment_count - 1)]

    # Создаем мульти-тело змейки
    snake_id = p.createMultiBody(
        baseMass=base_mass,
        baseCollisionShapeIndex=base_collision_shape_index,
        baseVisualShapeIndex=base_visual_shape_index,
        basePosition=base_position,
        baseOrientation=base_orientation,
        linkMasses=link_masses,
        linkCollisionShapeIndices=link_collision_shape_indices,
        linkVisualShapeIndices=link_visual_shape_indices,
        linkPositions=link_positions,
        linkOrientations=link_orientations,
        linkInertialFramePositions=link_inertial_frame_positions,
        linkInertialFrameOrientations=link_inertial_frame_orientations,
        linkParentIndices=link_parent_indices,
        linkJointTypes=link_joint_types,
        linkJointAxis=link_joint_axis
    )

    base_pos, _ = p.getBasePositionAndOrientation(snake_id)
    p.addUserDebugText("Base", base_pos, textColorRGB=[1, 0, 0], textSize=1.5)

    # Для каждого звена (сустава) получим позицию и нарисуем отладочную метку
    for i in range(segment_count - 1):
        # getLinkState возвращает, среди прочего, позицию точки крепления сустава в мировых координатах
        joint_state = p.getLinkState(snake_id, i, computeForwardKinematics=True)
        joint_pos = joint_state[0]  # позиция звена (это можно интерпретировать как место расположения сустава)

        # Отобразим текст с индексом сустава
        p.addUserDebugText(f"Joint {i}", joint_pos, textColorRGB=[0, 1, 0], textSize=1)

        # Можно также нарисовать небольшую линию, например, от центра звена вдоль оси X, чтобы показать ориентацию
        # Получим ориентацию сустава (в виде кватерниона)
        joint_orient = joint_state[1]
        # Определим направление оси X в мировых координатах
        axis_dir = p.multiplyTransforms([0, 0, 0], joint_orient, [0.1, 0, 0], [0, 0, 0, 1])[0]
        end_point = [joint_pos[j] + axis_dir[j] for j in range(3)]
        p.addUserDebugLine(joint_pos, end_point, lineColorRGB=[0, 0, 1], lineWidth=2)

    joints = list(range(segment_count - 1))
    return snake_id, joints
