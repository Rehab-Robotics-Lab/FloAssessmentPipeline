import numpy as np
import pykalman
from post_processing.src import game_tools
from post_processing.src import hampel_filter
from pose.src import joints


def get_times(hdf5_file, games, game_idx, time_name):
    times = {}
    times['start_t'] = games['start'][game_idx]
    times['finish_t'] = games['finish'][game_idx]
    times['start_idx'] = np.argmax(hdf5_file[time_name][:] > times['start_t'])
    times['finish_idx'] = np.argmax(
        hdf5_file[time_name][:] > times['finish_t'])
    return times


def filter_file(hdf5_file):
    games = game_tools.parse_games(hdf5_file)
    results = [None]*len(games['type'])
    for game_idx, _ in enumerate(games['type']):
        print(f'working on game {game_idx}')
        results[game_idx] = {}
        game_type = games['type'][game_idx]
        results[game_idx]['game_type'] = game_type
        if game_type == 'simon_says':
            time_name = 'vid/lower/color/time'
            keypoint_name = 'vid/lower/pose/openpose:25B/keypoints-median5/3d-realsense-raw'
            times = get_times(hdf5_file, games, game_idx, time_name)
            game_time = hdf5_file[time_name][times['start_idx']
                :times['finish_idx']]
            keypoint_data = hdf5_file[keypoint_name][times['start_idx']                                                     :times['finish_idx']]
            results[game_idx]['state'] = filter_ss(keypoint_data, game_time)
    return results


def filter_ss(keypoint_data, game_time):
    print('filtering a simon says game')
    filtered_state = {}
    filtered_state['time'] = game_time
    # start with wrists
    filtered_state['smoothed'] = {}
    filtered_state['raw'] = {}
    for joint_name in ['RWrist', 'LWrist']:
        print(f'working on {joint_name}')
        joint = joints.get_openpose_joint(joint_name)
        game_pose = keypoint_data[:, joint, :]
        raw_poses_filtered, state = filter_wrists_ss(game_pose, game_time)
        filtered_state['smoothed'][joint_name] = state
        filtered_state['raw'][joint_name] = raw_poses_filtered
    # process elbows

    # process shoulders
    return filtered_state


def filter_wrists_ss(game_pose, game_time):
    # for more insights on how this is being done, refer to notebooks/simon_says_filtering.ipynb

    # Measurement function (we measure x, y, and z positions)
    H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0],
                  ])

    # Observation covariance
    R = [[30**2,     0,     0],
         [0,     30**2,     0],
         [0,         0, 80**2]]

    # Transistion covariance - assume velocity only changes due to acceleration
    # and position only changes due to velocity
    Q = [np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 1]
                   ])*((100000*dt)**2) for dt in np.diff(game_time)]
    # State transistion matrix
    Fs = [np.array(
        [[1, 0, 0, dt,  0,  0, 1/2*(dt**2),           0,           0],
         [0, 1, 0,  0, dt,  0,           0, 1/2*(dt**2),           0],
         [0, 0, 1,  0,  0, dt,           0,           0, 1/2*(dt**2)],
         [0, 0, 0,  1,  0,  0,          dt,           0,           0],
         [0, 0, 0,  0,  1,  0,           0,          dt,           0],
         [0, 0, 0,  0,  0,  1,           0,           0,          dt],
         [0, 0, 0,  0,  0,  0,           1,           0,           0],
         [0, 0, 0,  0,  0,  0,           0,           1,           0],
         [0, 0, 0,  0,  0,  0,           0,           0,           1]
         ]) for dt in np.diff(game_time)]
    print('Filtering with Hampel Filter')
    bad_vals = np.unique(np.concatenate([hampel_filter.hampel_filter(
        game_pose[:, axis], 50, 5) for axis in range(3)], axis=0))
    game_pose[bad_vals, :] = np.nan
    print('Smoothing with Kalman Smoother')
    kalman_filter = pykalman.KalmanFilter(
        transition_matrices=Fs, observation_matrices=H,
        observation_covariance=R, transition_covariance=Q)
    game_pose_masked = np.ma.array(game_pose)
    game_pose_masked[np.isnan(game_pose_masked)] = np.ma.masked
    # Initial vals
    init_meds = np.ma.median(game_pose_masked[:20], axis=0)
    kalman_filter.initial_state_mean = np.array(
        [init_meds[0], init_meds[1], init_meds[2], 0, 0, 0, 0, 0, 0])
    computed_state, _ = kalman_filter.smooth(game_pose_masked)
    return (game_pose_masked, computed_state)
