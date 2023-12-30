class RLPilot:
    def __init__(self, log_dir):
        pass

        # Initialize ROS node, load policy network
        # set up subscribers and publishers

    def reset_queue(self):
        # set all queue elements to zero
        pass

    def prepare_inputs(self):
        # zero the input first

        # convert the observations to match the data spec required by the policy network

        return observation

    def update_queue(self, curr_state):
        # update the observation queue

        self.input_queue.append(observation)

    def update_gate_progress(self, curr_state):
        pass

        # check if we passed a racing gate

    def print_rollout_summary(self, curr_state):
        # this function is called when we completed a race

        rospy.signal_shutdown("Completed experiment")

    def save_rollout_to_csv(self):
        # save all race data to a csv file
        pass

    def odom_vio_callback(self, data):
        vio_state = copy.deepcopy(data)
        self.generate_action(vio_state)

    def generate_action(self, vio_state):
        pass

        # assemble policy network observation

        # Perform forward pass through policy network

        # Parse policy network output to generate a control command.

        # Publish control command

        # Update gate progress (check if we passed a racing gate)

    def start_callback(self, data):
        print("Start publishing commands!")
        # Start racing


if __name__ == "__main__":
    # generate pilot from saved checkpoint
    rl_pilot = RLPilot(log_dir=log_dir)
    rospy.spin()
