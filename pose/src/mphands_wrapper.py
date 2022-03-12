"""Module to extract hand poses using MediaPipe"""

import numpy as np
import cv2
from tqdm import tqdm
import mediapipe as mp


def process_frames(images):
    """process_frames.

    Args:
        images: An array of images to process on.

    Return: a dictionary of:
            keypoints
             - right
                - keypoints/color: color keypoints
                - 3dkeypoints/mp-world: the world coordinates predictated by mediapipe
                - confidence: The hand (not keypoint) detection confidence
             - left
                - same as right...
    """
    # left_hand (0-20) right hand (21-42)
    # dimmensions: x (camera), y (camera), z(camera), x (world), y (world), z (world)
    num_images = images.shape[0]
    keypoints = {'right': {'keypoints/color': np.zeros((num_images, 21, 3)),
                           'keypoints/mp-world': np.zeros((num_images, 21, 3)),
                           'confidence': np.zeros((num_images))},
                 'left': {'keypoints/color': np.zeros((num_images, 21, 3)),
                          'keypoints/mp-world': np.zeros((num_images, 21, 3)),
                          'confidence': np.zeros((num_images))}}
    mp_hands = mp.solutions.hands
    with mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=1,
            min_detection_confidence=0,
            min_tracking_confidence=0.5) as hands:
        for frame_id, frame in enumerate(tqdm(images, desc='frames')):
            # image = cv2.flip(cv2.imread(file), 1)
            # handedness is backwards (First person point of view)
            results = hands.process(
                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if results.multi_handedness:
                for hand_idx, hand_info in enumerate(results.multi_handedness):
                    side = hand_info.classification[0].label
                    landmarks = \
                        results.multi_hand_landmarks[hand_idx].landmark
                    landmarks_array = np.array(
                        [[landmark.x, landmark.y, landmark.z] for landmark in landmarks])
                    # scale x and z by image width
                    landmarks_array[:, (0, 2)] = \
                        landmarks_array[:, (0, 2)]*frame.shape[1]
                    # scale y by image height
                    landmarks_array[:, 1] = \
                        landmarks_array[:, 1]*frame.shape[0]
                    world_landmarks = \
                        results.multi_hand_world_landmarks[hand_idx].landmark

                    side_idx = 'right' if side == "Left" else "left"
                    keypoints[side_idx]['keypoints/color'][frame_id, :, :] = \
                        landmarks_array
                    keypoints[side_idx]['keypoints/mp-world'][frame_id, :, :] = np.array(
                        [[landmark.x, landmark.y, landmark.z] for landmark in world_landmarks])
                    keypoints[side_idx]['confidence'][frame_id] = \
                        hand_info.classification[0].score

    return keypoints
