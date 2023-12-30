class ActorNetRacing(network.DistributionNetwork):
    def __init__(self, observation_spec, action_spec, config, name="ActorNetwork"):
        self.config = config
        # define network architecture
        # the network architecture can be found in the paper and the config yaml files
        self._dense_layers = dense_layers
        self._projection_networks = projection_networks
        self._output_tensor_spec = action_spec

    @property
    def output_tensor_spec(self):
        return self._output_tensor_spec

    def call(self, observations, step_type, network_state, training=False, mask=None):
        # perform network forward pass
        return output_actions, network_state
