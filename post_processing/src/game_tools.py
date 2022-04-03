import json
import numpy as np


def parse_games(hdf5_file):
    game_start_idx = np.where(
        hdf5_file['/game_runner/commands/data'][:] == b'start')
    game_finish_idx = np.where(
        hdf5_file['/game_runner/commands/data'][:] == b'finish_game')
    games = {}
    games['type'] = [json.loads(arr)['game_type']
                     for arr in hdf5_file['/game_runner/def/data']]
    games['type_time'] = hdf5_file['/game_runner/def/time'][:]
    games['start'] = hdf5_file['/game_runner/commands/time'][game_start_idx]
    games['finish'] = hdf5_file['/game_runner/commands/time'][game_finish_idx]

    # make sure that each end falls between the previous and next start
    num_games = len(games['type'])
    for idx in range(num_games):
        if games['start'][idx] < games['type_time'][idx]:
            # this start is not the right one. add a start at the game load time
            print('inserted an extra start')
            games['start'] = np.insert(games['start'], idx, games['type_time'])
        if idx+1 > len(games['finish']):
            # we have already fixed everything else, just missing an end.
            # Just find the last time an action was played and call it good
            # enough (will cut off the last command)...
            print('added an end to the end')
            last_command_idx = np.where(
                hdf5_file['/game_runner/commands/data'][:] == b'next')[0][-1]
            last_command_time = hdf5_file['/game_runner/commands/time'][last_command_idx]
            games['finish'] = np.append(games['finish'], last_command_time)
        # check that finish comes after the expected start and before the next start
        if games['finish'][idx] <= games['start'][idx]:
            raise RuntimeError
        # if this is the last item, only check previous one
        if idx+1 == len(games['start']):
            continue
        if games['finish'][idx] >= games['start'][idx+1]:
            raise RuntimeError
    return games
