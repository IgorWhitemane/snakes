from utils.snake_env import SnakeEnv
from utils.callbacks import TensorboardCallback

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv


def train_model():
    env = DummyVecEnv([lambda: SnakeEnv(render_mode="direct")])
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.0003,
                batch_size=64, tensorboard_log="./logs/")
    model.learn(total_timesteps=100000, callback=TensorboardCallback())
    model.save("snake_ppo")
    env.close()

if __name__ == "__main__":
    train_model()
