def quat_to_euler(quat):
    """
    Convert quaternion to Euler angles.
    :param quat:
    :return:
    """
    quat = np.array(quat)

    if np.sum(quat) < 0.5:
        quat = np.array([1.0, 0.0, 0.0, 0.0])

    if len(quat.shape) == 1:
        quat = np.expand_dims(quat, axis=0)
    rot = R.from_quat(np.concatenate([np.expand_dims(quat[:, 1], axis=1),
                                      np.expand_dims(quat[:, 2], axis=1),
                                      np.expand_dims(quat[:, 3], axis=1),
                                      np.expand_dims(quat[:, 0], axis=1)], axis=1))

    euler_angles = rot.as_euler('xyz', degrees=False)

    return euler_angles


def quat_to_euler_vec(quat):
    """
    Convert quaternion to Euler angles.
    :param quat:
    :return:
    """

    rot = R.from_quat(np.concatenate([np.expand_dims(quat[:, 1], axis=1),
                                      np.expand_dims(quat[:, 2], axis=1),
                                      np.expand_dims(quat[:, 3], axis=1),
                                      np.expand_dims(quat[:, 0], axis=1)], axis=1))

    euler_angles = rot.as_euler('xyz', degrees=False)

    return euler_angles
