import pybullet as p

def create_snake_pybullet_2(segment_count=10, segment_radius=0.1, start_position=(0, 0, 1)):
    # Создаем голову змейки (базу мульти-тела)
    head_radius = segment_radius * 1.1
    base_collision_shape_index = p.createCollisionShape(p.GEOM_SPHERE, radius=head_radius)
    base_visual_shape_index = p.createVisualShape(p.GEOM_SPHERE, radius=head_radius, rgbaColor=[0.8, 0.7, 0.3, 1])
    base_mass = 1
    base_position = start_position
    base_orientation = [0, 0, 0, 1]

    # Для остальных сегментов зададим одинаковые параметры
    link_masses = [1] * (segment_count - 1)
    link_collision_shape_indices = [p.createCollisionShape(p.GEOM_SPHERE, radius=segment_radius)
                                    for _ in range(segment_count - 1)]
    link_visual_shape_indices = [p.createVisualShape(p.GEOM_SPHERE, radius=segment_radius,
                                                     rgbaColor=[0, 0.8, 0, 1])
                                 for _ in range(segment_count - 1)]

    # Позиция следующего звена относительно родительского звена
    link_positions = []
    link_orientations = []
    link_inertial_frame_positions = []
    link_inertial_frame_orientations = []
    link_parent_indices = []
    link_joint_types = []
    link_joint_axis = []

    # Будем располагать звенья вдоль оси X (плюс смещение)
    shift = 2 * segment_radius * 1.1
    for i in range(segment_count - 1):
        # Каждое звено расположено относительно предыдущего на расстоянии shift вдоль X
        link_positions.append([shift, 0, 0])
        link_orientations.append([0, 0, 0, 1])
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        # Родительский индекс: первое звено имеет индекс 0 (голова), далее 1, 2, ...
        link_parent_indices.append(i)
        # Используем револьвентный сустав, который позволяет поворачивать звено относительно оси Z
        link_joint_types.append(p.JOINT_REVOLUTE)
        link_joint_axis.append([0, 0, 1])

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

    # Получаем список суставов: их индексы от 0 до segment_count-2
    joints = list(range(segment_count - 1))

    # Возвращаем идентификатор мульти-тела, а также список суставных индексов
    # В дальнейшем для управления головой можно использовать snake_id, а для суставов – индексы из joints.
    return snake_id, joints
