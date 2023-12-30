def transform_to_world_frame(att, vec):
    """
    Transforms a vector expressed in body frame into the world frame.
    :param att: quaternion (w,x,y,z), np.array of shape (4,)
    :param vec:
    :return:
    """
    q_att = pyquaternion.Quaternion(w=att[0], x=att[1], y=att[2], z=att[3])

    vec_world = q_att.rotate(np.squeeze(vec))
    return vec_world


def transform_to_body_frame(att, vec):
    """
    Transforms a vector expressed in world frame into the body frame.
    :param att: quaternion (w,x,y,z), np.array of shape (4,)
    :param vec:
    :return:
    """
    q_att = pyquaternion.Quaternion(w=att[0], x=att[1], y=att[2], z=att[3])

    vec_body = q_att.inverse.rotate(np.squeeze(vec))
    return vec_body


def quat_multiply(q, r):
    """

    :param q:
    :param r:
    :return:
    """
    qw, qx, qy, qz = np.split(q, 4, axis=-1)
    rw, rx, ry, rz = np.split(r, 4, axis=-1)
    return np.concatenate(
        (rw * qw - rx * qx - ry * qy - rz * qz,
         rw * qx + rx * qw - ry * qz + rz * qy,
         rw * qy + rx * qz + ry * qw - rz * qx,
         rw * qz - rx * qy + ry * qx + rz * qw),
        axis=-1)


def quat_transpose(quat_in):
    """

    :param quat_in:
    :return:
    """
    w0, x0, y0, z0 = np.split(quat_in, 4, axis=-1)
    return np.concatenate([w0, -x0, -y0, -z0], axis=-1)


def clip_dataframe(df, gates, start_gate_idx, end_gate_idx):
    """

    :param df:
    :param gates:
    :param start_gate_idx:
    :param end_gate_idx:
    :return:
    """
    df_clipped = pd.DataFrame(columns=df.columns)

    curr_gate_idx = 0

    for i in range(len(df)):
        if 'gt_px' in df.columns:
            pos = np.array([df['gt_px'].values[i],
                            df['gt_py'].values[i],
                            df['gt_pz'].values[i]])
        else:
            pos = np.array([df['px'].values[i],
                            df['py'].values[i],
                            df['pz'].values[i]])
        p_G = gates[curr_gate_idx % len(gates)].att.inverse.rotate(pos - gates[curr_gate_idx % len(gates)].pos)
        if p_G[0] > 0.05:
            curr_gate_idx += 1

        if start_gate_idx <= curr_gate_idx < end_gate_idx:
            df_clipped = df_clipped.append(df.iloc[i])

    return df_clipped


def get_fixed_mins_maxs(mins, maxs):
    """

    :param mins:
    :param maxs:
    :return:
    """
    deltas = (maxs - mins) / 1.
    mins = mins - deltas * 0.25
    maxs = maxs + deltas * 0.25

    return [mins, maxs]


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')
