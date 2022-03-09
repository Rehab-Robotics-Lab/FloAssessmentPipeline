'''The joints that are used by the different pose detection
algorithms'''


def openpose_joints():
    """Return a list with the joints produced by openpose
    """
    return[
        "Nose",
        "LEye",
        "REye",
        "LEar",
        "REar",
        "LShoulder",
        "RShoulder",
        "LElbow",
        "RElbow",
        "LWrist",
        "RWrist",
        "LHip",
        "RHip",
        "LKnee",
        "RKnee",
        "LAnkle",
        "RAnkle",
        "UpperNeck",
        "HeadTop",
        "LBigToe",
        "LSmallToe",
        "LHeel",
        "RBigToe",
        "RSmallToe",
        "RHeel"
    ]


def mphands_joints():
    """Return a list with the joints produced by the mediapipe
    hands algorithm
    """
    return[
        'WRIST',
        'THUMB_CMC',
        'THUMB_MCP',
        'THUMB_IP',
        'THUMB_TIP',
        'INDEX_FINGER_MCP',
        'INDEX_FINGER_PIP',
        'INDEX_FINGER_DIP',
        'INDEX_FINGER_TIP',
        'MIDDLE_FINGER_MCP',
        'MIDDLE_FINGER_PIP',
        'MIDDLE_FINGER_DIP',
        'MIDDLE_FINGER_TIP',
        'RING_FINGER_MCP',
        'RING_FINGER_PIP',
        'RING_FINGER_DIP',
        'RING_FINGER_TIP',
        'PINKY_MCP',
        'PINKY_PIP',
        'PINKY_DIP',
        'PINKY_TIP'
    ]
