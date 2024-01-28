# from ding.policy import DQNPolicy

# # 初始化 DQN 策略
# dqn_policy = DQNPolicy(
#     model=your_model,  # 你的 Q 网络模型
#     gamma=0.99,        # 折扣因子
#     lr=1e-3            # 学习率
# )

# from ding.data.buffer import NaiveReplayBuffer

# # 初始化经验回放缓冲区
# buffer_size = 10000  # 定义缓冲区的大小
# replay_buffer = NaiveReplayBuffer(buffer_size)


# from ding.worker import Collector, SampleCollector

# # 初始化数据收集器
# collector = Collector(policy=dqn_policy, env=your_env)
# sample_collector = SampleCollector(collector, replay_buffer)

# # 收集数据
# new_data = sample_collector.collect(n_episode=1)  # 收集一定数量的完整回合数据

# from ding.worker import Evaluator

# # 初始化评估器
# evaluator = Evaluator(policy=dqn_policy, env=your_env)

# # 进行评估
# evaluator.eval(n_episode=10)  # 评估策略在10个回合中的表现


# from ding.config import compile_config

# # 配置管理器通常在整个训练的开始阶段使用
# cfg = """
# base_config:
#   env: your_env_name
#   policy: dqn
#   replay_buffer_size: 10000
# """
# # 编译配置
# compiled_cfg = compile_config(cfg)



# # 假设所有的配置和初始化都已完成

# # 训练循环
# for epoch in range(num_epochs):
#     # 数据收集
#     new_data = sample_collector.collect(n_episode=collect_episodes_per_epoch)

#     # 数据处理并存储到 ReplayBuffer
#     replay_buffer.push(new_data)

#     # 训练
#     for _ in range(update_times_per_epoch):
#         batch_data = replay_buffer.sample(batch_size)  # 从缓冲区中抽取一批数据
#         dqn_policy.learn(batch_data)  # 使用抽取的数据更新策略

#     # 定期评估模型性能
#     if epoch % eval_interval == 0:
#         eval_result = evaluator.eval(n_episode=eval_episodes)
#         print(f"Evaluation result at epoch {epoch}: {eval_result}")


import gym
from ditk import logging
from ding.model import DQN
from ding.policy import DQNPolicy
from ding.envs import DingEnvWrapper, BaseEnvManagerV2, SubprocessEnvManagerV2
from ding.data import DequeBuffer
from ding.config import compile_config
from ding.framework import task, ding_init
from ding.framework.context import OnlineRLContext
from ding.framework.middleware import OffPolicyLearner, StepCollector, interaction_evaluator, data_pusher, \
    eps_greedy_handler, CkptSaver, online_logger, nstep_reward_enhancer
from ding.utils import set_pkg_seed
from dizoo.box2d.lunarlander.config.lunarlander_dqn_config import main_config, create_config

def main():
    # 设置日志记录级别
    logging.getLogger().setLevel(logging.INFO)

    # 编译配置文件，将用户定义的配置和系统默认配置合并
    cfg = compile_config(main_config, create_cfg=create_config, auto=True)

    # 初始化任务，包括日志、设备等环境设置
    ding_init(cfg)

    # 使用上下文管理器启动任务，这里的任务指强化学习的训练和评估流程
    with task.start(async_mode=False, ctx=OnlineRLContext()):
        # 创建环境管理器，用于并行管理多个环境实例
        collector_env = SubprocessEnvManagerV2(
            env_fn=[lambda: DingEnvWrapper(gym.make(cfg.env.env_id)) for _ in range(cfg.env.collector_env_num)],
            cfg=cfg.env.manager
        )

        # 创建评估器环境管理器
        evaluator_env = SubprocessEnvManagerV2(
            env_fn=[lambda: DingEnvWrapper(gym.make(cfg.env.env_id)) for _ in range(cfg.env.evaluator_env_num)],
            cfg=cfg.env.manager
        )

        # 设置随机种子以获得可复现的结果
        set_pkg_seed(cfg.seed, use_cuda=cfg.policy.cuda)

        # 初始化DQN模型
        model = DQN(**cfg.policy.model)

        # 创建经验回放缓冲区
        buffer_ = DequeBuffer(size=cfg.policy.other.replay_buffer.replay_buffer_size)

        # 创建DQN策略
        policy = DQNPolicy(cfg.policy, model=model)

        # 使用中间件构建强化学习的各个组件
        task.use(interaction_evaluator(cfg, policy.eval_mode, evaluator_env))
        task.use(eps_greedy_handler(cfg))
        task.use(StepCollector(cfg, policy.collect_mode, collector_env))
        task.use(nstep_reward_enhancer(cfg))
        task.use(data_pusher(cfg, buffer_))
        task.use(OffPolicyLearner(cfg, policy.learn_mode, buffer_))
        task.use(online_logger(train_show_freq=10))
        task.use(CkptSaver(policy, cfg.exp_name, train_freq=100))

        # 运行任务，执行训练
        task.run()

# 程序入口
if __name__ == "__main__":
    main()