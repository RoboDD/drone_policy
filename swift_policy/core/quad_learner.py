class QuadLearner:
    def __init__(self, settings):
        self.config = settings
        self.generate_wandb_log = True
        self.use_tf_functions = True

        if self.config.resume_training:
            print("Resuming training from previous checkpoint.")
            # Load checkpoint from disk

        self.summary_writer = None
        if self.config.rl_mode == RlMode.TRAIN:
            # save training parameters to disk

        print("=============================")
        print("Setting hyperparameters")
        print("=============================")

        # Set training hyperparameters
        # - PPO parameters
        # - track layout
        # - network architecture flags

        print("=============================")
        print("Setting environment")
        print("=============================")

        # Instantiate the training and evaluation environments.
        # Both environments share the same physics simulation and 
        # simulate the same number of agents. The eval environment
        # always executes the greedy action of the policy.
        self.collect_env, self.eval_env = createEnvironment(config=self.config)

        ## Distribution strategy (CPU/GPU/TPU)
        print("=============================")
        print("Setting distribution strategy")
        print("=============================")
        use_gpu = self.config.gpu_device >= 0

        print("=============================")
        print("Setting agent")
        print("=============================")

        # Instantiate the training agent.
        self.agent = createAlgorithm(config=self.config, collect_env=self.collect_env, train_step=self.train_step)
        print("Initializing agent...")
        self.agent.initialize()

        eval_policy = self.agent.policy
        collect_policy = self.agent.collect_policy

        # set up replay buffer: we use a uniform replay buffer


    def train(self):
        print("============================")
        print("Entering main training loop!")
        print("============================")

        # Main training loop
        for train_iter in range(self.config.num_iterations):
            # we perform training for a fixed number of environment interactions
            if training_finished:
                print("Completed training!")
                return

            # collect data (fixed number of environment steps) by running the collect driver

            # Perform train step on collected data
            for _ in range(self.config.algorithm_params.train_steps_per_iter):
                train_loss = self.train_step_fn()

            # collect rollout statistics
            if train_iter % self.config.eval_every_nth == 0:
                # Perform test run
                # Log eval metrics
                pass
            # Clear the replay buffer

    def test(self, traj_dir=None, slider=1.0):
        print("==============================================================")
        print("Testing policy: \n - [%s]" % self.config.log_dir)
        print("==============================================================")
        # Test a trained policy in simulation and collect test metrics. 

        print("================================================================")
        print("Testing complete")
        print("================================================================")
        return test_metrics

    def identify_residual_models(self):
        # Identify residual models from data collected in the real world.
        self.identify_residual_observation()
        self.identify_residual_dynamics()

    def identify_residual_dynamics(self):
        print("Identifying residual dynamics model...")
        dyn_adaptation = DynamicsAdaptation(self.config)
        dyn_adaptation.identify_residual_dynamics()

    def identify_residual_observation(self):
        print("Identifying residual observation model...")
        obs_adaptation = ObservationAdaptation(self.config)
        obs_adaptation.identify_residual_observation()
