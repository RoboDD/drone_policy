def createEnvironment(config):
    # create collect and eval environments to be used to train or finetune a policy.

    return collect_env, eval_env


def createTestEnvironment(config):
    # create a test environment for evaluation

    return test_env
