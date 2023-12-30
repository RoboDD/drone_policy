class Quad3dEnvCPPVec(py_environment.PyEnvironment):
    def __init__(self, config, create_logs=False, is_test_env=False, verbose=False):
        # Initialize parallelized simulation environment, pass platform & reward properties.
        # The simulation itself is based on our quadrotor control stack called 'Agilicious':
        # https://rpg.ifi.uzh.ch/docs/ScienceRobotics22_Foehn.pdf
        # The code will be available under https://github.com/uzh-rpg/agilicious
        # The simulation of 'Agilicious' is extended to simulate N=100 quadrotors simultaneously using
        # OpenMP, which substantially accelerates data collection.
 
        # Define observation and action tensor specs.
        # We use TF-Agents' nested tensor format to describe network-input and -output shapes. 
        pass

    @property
    def batched(self) -> bool:
        return True

    @property
    def batch_size(self):
        return self.num_envs

    def _reset(self):
        # reset all environments
        # either initializes the drone at the start podium or in a random gate on the track
        return time_step_vec

    def _step(self, action):
        # apply the predicted actions to all environments, simulate for one timestep, compute reward
        return time_step_vec
