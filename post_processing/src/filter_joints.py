import numpy as np
import pykalman
from post_processing.src import game_tools
from post_processing.src import hampel_filter
from pose.src import joints
import ipdb


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
            pixel_keypoint_name = 'vid/lower/pose/openpose:25B/keypoints-median5/color'
            times = get_times(hdf5_file, games, game_idx, time_name)
            # it is possible that little to none of the game was recorded. handle that
            if (times['finish_idx']-times['start_idx']) < 100:
                print('game with little to no recording, passing')
                continue
            game_time = hdf5_file[time_name][times['start_idx']                                             :times['finish_idx']]
            keypoint_data = hdf5_file[keypoint_name][times['start_idx']                                                     :times['finish_idx']]
            pixel_data = hdf5_file[pixel_keypoint_name][times['start_idx']                                                        :times['finish_idx']]
            results[game_idx]['state'] = filter_ss(
                keypoint_data, pixel_data, game_time)
    return results

# TODO: If the begining and/or end are all NAN, then results can drift.
#       The best thing to do is probably to trim the data for all joints
#       of interest to the first place where there is a valid value and
#       do the same for the end


def filter_ss(keypoint_data, pixel_data, game_time):
    print('filtering a simon says game')
    filtered_state = {}
    filtered_state['time'] = game_time
    # start with wrists
    filtered_state['smooth'] = {}
    filtered_state['raw'] = {}
    filtered_state['filtered'] = {}
    filtered_state['covariance'] = {}
    for side in ['R', 'L']:
        wrist_name = f'{side}Wrist'
        print(f'working on {wrist_name}')
        wrist_idx = joints.get_openpose_joint(wrist_name)
        wrist_pose = keypoint_data[:, wrist_idx, :]
        if np.all(np.isnan(wrist_pose)):
            continue
        wrist_poses_filtered, wrist_state, wrist_state_covariance = filter_wrist(
            wrist_pose, game_time)
        filtered_state['smooth'][wrist_name] = wrist_state
        filtered_state['filtered'][wrist_name] = wrist_poses_filtered
        filtered_state['raw'][wrist_name] = wrist_pose
        filtered_state['covariance'][wrist_name] = wrist_state_covariance
        # process elbows
        elbow_name = f'{side}Elbow'
        print(f'working on {elbow_name}')
        elbow_idx = joints.get_openpose_joint(elbow_name)
        elbow_pose = keypoint_data[:, elbow_idx, :]
        elbow_pixels = pixel_data[:, elbow_idx, :2]
        wrist_pixels = pixel_data[:, wrist_idx, :2]
        if np.all(np.isnan(elbow_pose)):
            continue
        elbow_poses_filtered, elbow_state, elbow_state_covariance = filter_elbow(
            elbow_pose, wrist_state[:, 2], game_time, elbow_pixels, wrist_pixels)
        filtered_state['smooth'][elbow_name] = elbow_state
        filtered_state['filtered'][elbow_name] = elbow_poses_filtered
        filtered_state['raw'][elbow_name] = elbow_pose
        filtered_state['covariance'][elbow_name] = elbow_state_covariance
        # process shoulders
        shoulder_name = f'{side}Shoulder'
        print(f'working on {shoulder_name}')
        shoulder_idx = joints.get_openpose_joint(shoulder_name)
        shoulder_pose = keypoint_data[:, shoulder_idx, :]
        shoulder_pixels = pixel_data[:, shoulder_idx, :2]
        if np.all(np.isnan(shoulder_pose)):
            continue
        shoulder_poses_filtered, shoulder_state, shoulder_state_covariance = filter_shoulder(
            shoulder_pose, wrist_state[:, 2], game_time,
            shoulder_pixels, elbow_pixels, wrist_pixels)
        filtered_state['smooth'][shoulder_name] = shoulder_state
        filtered_state['filtered'][shoulder_name] = shoulder_poses_filtered
        filtered_state['raw'][shoulder_name] = shoulder_pose
        filtered_state['covariance'][shoulder_name] = shoulder_state_covariance
    return filtered_state


def filter_shoulder(game_pose, wrist_depth, game_time, shoulder_pixels, elbow_pixels, wrist_pixels):
    # for more insights on how this is being done, refer to notebooks/simon_says_filtering.ipynb

    # Measurement function (we measure x, y, and z positions)
    H = np.array([[1, 0, 0, 0, 0, 0, ],
                  [0, 1, 0, 0, 0, 0, ],
                  [0, 0, 1, 0, 0, 0, ],
                  ])

    # Observation covariance
    # https://stackoverflow.com/a/9537830/5274985
    clean_depth = game_pose[:, 2]
    mask = np.isnan(clean_depth)
    clean_depth[mask] = np.interp(np.flatnonzero(
        mask), np.flatnonzero(~mask), clean_depth[~mask])
    R = [[[(0.0215*z)**2,             0,           0],
          [0,             (0.0215*z)**2,           0],
          [0,                         0, (0.04*z)**2]]
         for z in clean_depth]

    # Transistion covariance - assume constant velocity model with only slight changes
    # in velocity.
    Q = [np.array([[0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 1]
                   ])*((10*dt)**2) for dt in np.diff(game_time)]
    # State transistion matrix
    Fs = [np.array(
        [[1, 0, 0, dt,  0,  0],
         [0, 1, 0,  0, dt,  0],
         [0, 0, 1,  0,  0, dt],
         [0, 0, 0,  1,  0,  0],
         [0, 0, 0,  0,  1,  0],
         [0, 0, 0,  0,  0,  1]
         ]) for dt in np.diff(game_time)]

    # don't use values where the hand is in front of the elbow.
    # if x and y for wrist and elbow are at the same point, then mask that shit
    # let's say +/- 75 mm
    px_size = wrist_depth*1.375/1920
    margin_error = 75/px_size

    # TODO, what about entire forearm?
    # check if points inside box defined by wrist elbow +/- margin error
    x_high = np.max([wrist_pixels[:, 0], elbow_pixels[:, 0]],
                    axis=0)+margin_error
    x_low = np.min([wrist_pixels[:, 0], elbow_pixels[:, 0]],
                   axis=0)-margin_error
    y_high = np.max([wrist_pixels[:, 1], elbow_pixels[:, 1]],
                    axis=0)+margin_error
    y_low = np.min([wrist_pixels[:, 1], elbow_pixels[:, 1]],
                   axis=0)-margin_error
    in_box = (shoulder_pixels[:, 0] > x_low) & (shoulder_pixels[:, 0] < x_high) & (
        shoulder_pixels[:, 1] > y_low) & (shoulder_pixels[:, 1] < y_high)

    x_1 = wrist_pixels[:, 0]
    y_1 = wrist_pixels[:, 1]
    x_2 = elbow_pixels[:, 0]
    y_2 = elbow_pixels[:, 0]

    x_0 = shoulder_pixels[:, 0]
    y_0 = shoulder_pixels[:, 1]

    # hacky way to prevent division by zero
    same = (x_1 == x_2) & (y_1 == y_2)
    x_1[same] = x_1[same]+1
    if np.any(np.sqrt((x_2-x_1)**2 + (y_2-y_1)**2) == 0):
        ipdb.set_trace()
    dist = np.abs((x_2-x_1)*(y_1-y_0)-(x_1-x_0)*(y_2-y_1)) / \
        np.sqrt((x_2-x_1)**2 + (y_2-y_1)**2)

    game_pose[(dist < margin_error) & in_box, :] = np.nan

    print('Filtering with Hampel Filter')
    bad_vals = np.unique(np.concatenate([hampel_filter.hampel_filter_v(
        game_pose[:, axis], 50, 5) for axis in range(3)], axis=0))
    game_pose[bad_vals, :] = np.nan
    print('Smoothing with Kalman Smoother')
    kalman_filter = pykalman.KalmanFilter(
        transition_matrices=Fs, observation_matrices=H,
        observation_covariance=R, transition_covariance=Q)
    game_pose_masked = np.ma.array(game_pose)
    game_pose_masked[np.isnan(game_pose_masked)] = np.ma.masked
    # Initial vals
    init_meds = [np.ma.median(gp[~gp.mask][:20]) for gp in game_pose_masked.T]
    kalman_filter.initial_state_mean = np.array(
        [init_meds[0], init_meds[1], init_meds[2], 0, 0, 0])
    computed_state, state_covariance = kalman_filter.smooth(game_pose_masked)
    return (game_pose_masked, computed_state, state_covariance)


def filter_elbow(game_pose, wrist_depth, game_time, elbow_pixels, wrist_pixels):
    # for more insights on how this is being done, refer to notebooks/simon_says_filtering.ipynb

    # Measurement function (we measure x, y, and z positions)
    H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0],
                  ])

    # Observation covariance
    clean_depth = game_pose[:, 2]
    mask = np.isnan(clean_depth)
    clean_depth[mask] = np.interp(np.flatnonzero(
        mask), np.flatnonzero(~mask), clean_depth[~mask])
    R = [[[(0.0215*z)**2,             0,           0],
          [0,             (0.0215*z)**2,           0],
          [0,                         0, (0.04*z)**2]] for z in clean_depth]

    # Transistion covariance - assume velocity only changes due to acceleration
    # and position only changes due to velocity
    # probably elbow accceleration changes way slower than wrist?
    Q = [np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 1, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 1, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 1]
                   ])*((10000*dt)**2) for dt in np.diff(game_time)]
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

    # don't use values where the hand is in front of the elbow.
    # if x and y for wrist and elbow are at the same point, then mask that shit
    # let's say +/- 75 mm
    print('Eliminating wrist occlusions')
    px_size = wrist_depth*1.375/1920
    margin_error = 75/px_size
    wrist_elbow_dist = np.sqrt(
        np.sum(np.power(elbow_pixels-wrist_pixels, 2), axis=1))
    game_pose[wrist_elbow_dist < margin_error, :] = np.nan

    print('Filtering with Hampel Filter')
    bad_vals = np.unique(np.concatenate([hampel_filter.hampel_filter_v(
        game_pose[:, axis], 50, 5) for axis in range(3)], axis=0))
    game_pose[bad_vals, :] = np.nan
    print('Smoothing with Kalman Smoother')
    kalman_filter = pykalman.KalmanFilter(
        transition_matrices=Fs, observation_matrices=H,
        observation_covariance=R, transition_covariance=Q)
    game_pose_masked = np.ma.array(game_pose)
    game_pose_masked[np.isnan(game_pose_masked)] = np.ma.masked
    # Initial vals
    init_meds = [np.ma.median(gp[~gp.mask][:20]) for gp in game_pose_masked.T]
    kalman_filter.initial_state_mean = np.array(
        [init_meds[0], init_meds[1], init_meds[2], 0, 0, 0, 0, 0, 0])
    computed_state, state_covariance = kalman_filter.smooth(game_pose_masked)
    return (game_pose_masked, computed_state, state_covariance)


def filter_wrist(game_pose, game_time):
    # for more insights on how this is being done, refer to notebooks/simon_says_filtering.ipynb

    # Measurement function (we measure x, y, and z positions)
    H = np.array([[1, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0],
                  ])

    # Observation covariance
    # For x and y:
    #   tand(69/2)*2=1.375
    #   1.375/1920*15 = 0.0107
    #   x2 = 0.0215 (double for good measure, to handle z interaction)
    # For z:
    #   2*2%=0.04
    clean_depth = game_pose[:, 2]
    mask = np.isnan(clean_depth)
    clean_depth[mask] = np.interp(np.flatnonzero(
        mask), np.flatnonzero(~mask), clean_depth[~mask])
    R = [[[(0.0215*z)**2,             0,           0],
          [0,             (0.0215*z)**2,           0],
          [0,                         0, (0.04*z)**2]] for z in clean_depth]

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
    bad_vals = np.unique(np.concatenate([hampel_filter.hampel_filter_v(
        game_pose[:, axis], 50, 5) for axis in range(3)], axis=0))
    game_pose[bad_vals, :] = np.nan
    print('Smoothing with Kalman Smoother')
    kalman_filter = pykalman.KalmanFilter(
        transition_matrices=Fs, observation_matrices=H,
        observation_covariance=R, transition_covariance=Q)
    game_pose_masked = np.ma.array(game_pose)
    game_pose_masked[np.isnan(game_pose_masked)] = np.ma.masked
    # Initial vals
    init_meds = [np.ma.median(gp[~gp.mask][:20]) for gp in game_pose_masked.T]
    kalman_filter.initial_state_mean = np.array(
        [init_meds[0], init_meds[1], init_meds[2], 0, 0, 0, 0, 0, 0])
    computed_state, state_covariance = kalman_filter.smooth(game_pose_masked)
    return (game_pose_masked, computed_state, state_covariance)
