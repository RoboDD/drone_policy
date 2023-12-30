class ValueNetRacing(network.Network):
    def __init__(self, input_tensor_spec, config, name="ValueNetwork"):
        # define value network architecture
        # the network architecture can be found in the paper
        self._encoding_combiner = tf.keras.layers.Concatenate(axis=-1)
        self._dense_layers = dense_layers

    def call(self, observation, step_type=(), network_state=()):
        # perform forward pass of value network

        return tf.squeeze(value, -1), network_state
