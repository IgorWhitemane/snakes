from stable_baselines3.common.callbacks import BaseCallback

class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=1):
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        if "rewards" in self.locals and len(self.locals["rewards"]) > 0:
            episode_reward = sum(self.locals["rewards"])  # Суммируем награды за шаги
            self.logger.record("episode_reward", episode_reward)
        return True