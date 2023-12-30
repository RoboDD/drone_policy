class DynamicsAdaptation:
    def __init__(self, config=None):
        # Initialize the dynamics residual model identification.
        print("Initialized Dynamics Adaptation.")

    def identify_residual_dynamics(self):
        # load a set of racing runs.
        # Compute residual forces based on the logged commanded actions and the observed forces.
        # Assemble dataset of residual forces.
        # Perform identification:
        self._identify_residual_dynamics_impl(df_train, df_test)

    @staticmethod
    def _identify_residual_dynamics_impl(
        df_train, df_test, return_data=False, generate_plot=False
    ):
        # Prepare dataset

        # setup KNN regression with n_neighbors = 5
        n_neighbors = 5
        weights = "uniform"  # ["uniform", "distance"]
        knn_test = neighbors.KNeighborsRegressor(n_neighbors, weights=weights)
        knn_test.fit(test_features, test_labels)

        # save identified residual force dataset to disk.
        # The dataset will be loaded by the training environment during finetuning.
        knn_train_save_fname = (
            os.path.join(self.config.log_dir, "residual_dynamics")
            + "/residual_dynamics_train_knn.pickle"
        )

        with open(knn_train_save_fname, "wb") as handle:
            print("Saving KNN model to [%s]" % knn_train_save_fname)
            pickle.dump(knn_train, handle, protocol=pickle.HIGHEST_PROTOCOL)


class ObservationAdaptation:
    def __init__(self, config=None):
        # Initialize residual observation model identification.
        print("Initialized Observation Adaptation.")

    def identify_residual_observation(self):
        # Load a set of recorded racing runs.

        # Identify the observation residual from the saved VIO estimates and the
        # Vicon measurements.

        # Concatenate all data to a single identification dataset.

        # perform GP-fitting of the residual
        self._identify_residual_observation_impl(df_with_name_list, self.config)

        return len(df_with_name_list) > 0

    @staticmethod
    def _identify_residual_observation_impl(
        df_with_name_list, config, return_data=False, generate_plot=False
    ):
        # Define GP kernel types as a set of RBF kernels.
        kernel_px = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )
        kernel_py = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )
        kernel_pz = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )

        kernel_vx = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )
        kernel_vy = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )
        kernel_vz = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )

        kernel_attx = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )
        kernel_atty = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )
        kernel_attz = (
            ConstantKernel(1.0) * RBF(1.0)
            + ConstantKernel(0.2) * RBF(0.5)
            + ConstantKernel(0.1) * RBF(0.01)
        )

        # Create GP regressors
        gp_px = GaussianProcessRegressor(
            kernel=kernel_px, n_restarts_optimizer=3, alpha=0.05
        )
        gp_py = GaussianProcessRegressor(
            kernel=kernel_py, n_restarts_optimizer=3, alpha=0.05
        )
        gp_pz = GaussianProcessRegressor(
            kernel=kernel_pz, n_restarts_optimizer=3, alpha=0.05
        )

        position_kernels = [gp_px, gp_py, gp_pz]

        gp_vx = GaussianProcessRegressor(
            kernel=kernel_vx, n_restarts_optimizer=3, alpha=0.1
        )
        gp_vy = GaussianProcessRegressor(
            kernel=kernel_vy, n_restarts_optimizer=3, alpha=0.1
        )
        gp_vz = GaussianProcessRegressor(
            kernel=kernel_vz, n_restarts_optimizer=3, alpha=0.1
        )

        velocity_kernels = [gp_vx, gp_vy, gp_vz]

        gp_attx = GaussianProcessRegressor(
            kernel=kernel_attx, n_restarts_optimizer=3, alpha=0.001
        )
        gp_atty = GaussianProcessRegressor(
            kernel=kernel_atty, n_restarts_optimizer=3, alpha=0.001
        )
        gp_attz = GaussianProcessRegressor(
            kernel=kernel_attz, n_restarts_optimizer=3, alpha=0.001
        )

        attitude_kernels = [gp_attx, gp_atty, gp_attz]

        # Assemble dataset from recorded rollouts.

        # fit position kernels
        for axis_idx, position_kernel in enumerate(position_kernels):
            # concatenate features from dataset
            X_ind_x = ...
            # concatenate labels from dataset
            y_ind_x = ...
            # fit GP regressor
            position_kernel = position_kernel.fit(X_ind_x, y_ind_x)
            trained_kernel_list.append(copy.deepcopy(position_kernel))

            # sample realizations from the posterior for position
            sample_features = X_ind_x
            pos_sampled = position_kernel.sample_y(
                sample_features, n_samples, random_state=seed
            )
            pos_sampled = np.concatenate(
                [np.expand_dims(y_ind_x, axis=2), pos_sampled], axis=2
            )
            pos_sample_data.append(np.squeeze(pos_sampled, axis=1))
        trained_pos_kernels.append(copy.deepcopy(trained_kernel_list))

        # perform same procedure for velocity error regressors

        # perform same procedure for attitude error regressors

        # Store sampled realizations to disk.
        # They will be loaded by the training environment during finetuning.

        return
