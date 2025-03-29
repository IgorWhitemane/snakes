from utils.snake_env import SnakeEnv
from utils.callbacks import TensorboardCallback

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv


def train_model():
    env = DummyVecEnv([lambda: SnakeEnv(render_mode="human")])
    model = PPO("MlpPolicy",
                env, verbose=1,
                learning_rate=0.0001,
                n_steps=2048,
                batch_size=64,
                n_epochs=10,
                gae_lambda=0.95,
                clip_range=0.3,
                ent_coef=0.01,
                vf_coef=0.5,
                max_grad_norm=0.5,
                tensorboard_log="./logs/")

    model.learn(total_timesteps=3000000, callback=TensorboardCallback())
    model.save("snake_ppo")
    env.close()

if __name__ == "__main__":
    train_model()
