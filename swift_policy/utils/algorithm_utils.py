def createAlgorithm(config, collect_env, train_step):
    """

    :param config:
    :param collect_env:
    :param train_step:
    :return:
    """

    observation_spec, action_spec, time_step_spec = spec_utils.get_tensor_specs(
        collect_env
    )

    if config.algorithm == "PPO":
        actor_net = ActorNetRacing(
            observation_spec=observation_spec, action_spec=action_spec, config=config
        )
        value_net = ValueNetRacing(input_tensor_spec=observation_spec, config=config)

        tf_agent = ppo_agent.PPOAgent(
            # PPO hyperparams
        )

    return tf_agent
