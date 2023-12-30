class Track:
    def __init__(
        self,
        track_layout_yaml,
        gate_size=1.45,
        gate_depth=0.05,
        z_offset=0.0,
        yaw_offset=0.0,
    ):
        self.gates = []

        print("Loading track layout from [%s]" % track_layout_yaml)
        assert os.path.isfile(track_layout_yaml), track_layout_yaml
        with open(track_layout_yaml, "r") as stream:
            track_yaml = yaml.safe_load(stream)
            for i in range(track_yaml["gates"]["N"]):
                gate_str = "Gate" + str(i + 1)
                curr_gate = Gate(
                    track_yaml["gates"][gate_str]["position"],
                    track_yaml["gates"][gate_str]["rotation"],
                )
                self.gates.append(curr_gate)
