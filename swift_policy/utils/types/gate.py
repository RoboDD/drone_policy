class Gate:
    def __init__(self, pos, att, size=1.45):
        pass
        # initialize gate

    def getCorners(self):
        corners = np.concatenate(
            [
                self.tl_corner[:, np.newaxis],
                self.tr_corner[:, np.newaxis],
                self.br_corner[:, np.newaxis],
                self.bl_corner[:, np.newaxis],
            ],
            axis=1,
        )
        return corners

    def is_passed(self, pos: np.ndarray) -> bool:
        return self.att.inverse.rotate(pos - self.pos)[0] > 0.05

    def __repr__(self):
        return "Gate at [%.2f, %.2f, %.2f] with yaw %.2f deg." % (
            self.pos[0],
            self.pos[1],
            self.pos[2],
            180.0 / np.pi * self.yaw,
        )
