from stable_baselines3 import PPO
from utils.snake_env import SnakeEnv

# Загружаем обученную модель
model = PPO.load("snake_ppo.zip")

# Создаем окружение
env = SnakeEnv(render_mode="human")  # Убедись, что у тебя есть режим "human"
obs, _ = env.reset()

# Запускаем симуляцию
done = False
while not done:
    action, _ = model.predict(obs, deterministic=True)  # Делаем предсказание
    obs, reward, done, truncated, info = env.step(action)  # Выполняем шаг
    print(f"Action: {action}, Reward: {reward}")

# Закрываем окружение
env.close()
