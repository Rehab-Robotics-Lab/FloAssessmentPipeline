'''The joints that are used by the different pose detection algorithms'''

import numpy as np


def openpose_joints():
    """Return a list with the joints produced by openpose
    """
    return [
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


def get_openpose_joint(name):
    return np.where(np.asarray(openpose_joints()) == name)[0][0]


def openpose_joint_pairs():
    """Pairs of joints that are connected
    Joints listed here:
    https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/02_output.md#keypoints-in-cpython
    """
    return [
        (openpose_joints().index('UpperNeck'),
         openpose_joints().index('LShoulder')),
        (openpose_joints().index('UpperNeck'),
         openpose_joints().index('RShoulder')),
        (openpose_joints().index('LShoulder'),
            openpose_joints().index('LElbow')),
        (openpose_joints().index('RShoulder'),
            openpose_joints().index('RElbow')),
        (openpose_joints().index('LElbow'),
            openpose_joints().index('LWrist')),
        (openpose_joints().index('RElbow'),
            openpose_joints().index('RWrist')),
        (openpose_joints().index('UpperNeck'),
            openpose_joints().index('LHip')),
        (openpose_joints().index('UpperNeck'),
            openpose_joints().index('RHip'))
    ]


def openpose_upper_body_joints():
    """Joints (names) from openpose that are in the
    upper body
    """
    return [openpose_joints().index(key) for key in [
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
            "UpperNeck",
            "HeadTop"]]


def mphands_joints():
    """Return a list with the joints produced by the mediapipe
    hands algorithm
    """
    return [
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


def mphands_joint_pairs():
    """Pairs of joints that are connected
    Joints listed here:
    https://google.github.io/mediapipe/solutions/hands.html#hand-landmark-model
    """
    return [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 4),
        (0, 5),
        (5, 6),
        (6, 7),
        (7, 8),
        (0, 9),
        (9, 10),
        (10, 11),
        (11, 12),
        (0, 13),
        (13, 14),
        (14, 15),
        (15, 16),
        (0, 17),
        (17, 18),
        (18, 19),
        (19, 20)
    ]
