import pybullet as p
import pybullet_data


def setup_pybullet(render_mode="human"):
    """Настройка PyBullet."""
    if render_mode == "human":
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())