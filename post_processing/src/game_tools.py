import json
import numpy as np
import ipdb


def parse_games(hdf5_file):
    game_start_idx = np.where(
        hdf5_file['/game_runner/commands/data'][:] == b'start')
    game_finish_idx = np.where(
        hdf5_file['/game_runner/commands/data'][:] == b'finish_game')
    games = {}
    games['type'] = []
    games['type_time'] = []
    games['start'] = []
    games['finish'] = []
    # games['type'] = [json.loads(arr)['game_type']
    #                  for arr in hdf5_file['/game_runner/def/data']]
    # games['type_time'] = hdf5_file['/game_runner/def/time'][:]
    # games['start'] = hdf5_file['/game_runner/commands/time'][game_start_idx]
    # games['finish'] =

    # TODO: This is not at all clean

    # make sure that each end falls between the previous and next start
    # we want to find the first and last actions and work between those. Often times the game
    # is started well before any actions are occcuring and often the operator fails to stop
    # the game.

    # for each game definition, find the next next, that is the start
    # for each game definition, find the finish_game. If that finish_game is after the next definition,
    # then set the end as the next definition find the last next prior to that end and call that the
    # end of the game.
    # The first next tells the robot to explain the game. The last one says congrats.
    # there is often a lot of robot movement before and after that
    next_command_mask = hdf5_file['/game_runner/commands/data'][:] == b'next'
    next_times = hdf5_file['/game_runner/commands/time'][next_command_mask]
    finish_times = hdf5_file['/game_runner/commands/time'][game_finish_idx]
    game_types = [json.loads(arr)['game_type']
                  for arr in hdf5_file['/game_runner/def/data']]
    def_times = hdf5_file['/game_runner/def/time'][:]
    num_games = len(game_types)

    for idx in range(num_games):
        nexts_after_start = np.flatnonzero(
            def_times[idx] < next_times)
        if len(nexts_after_start) == 0:
            continue
        first_next = next_times[nexts_after_start[0]]
        nexts_finish_idx = np.flatnonzero(finish_times > first_next)
        nexts_def_idx = np.flatnonzero(def_times > first_next)
        if len(nexts_finish_idx) == 0 and len(nexts_def_idx) == 0:
            final_nexts = [-1]
        elif len(nexts_finish_idx) == 0:
            next_def = def_times[nexts_def_idx[0]]
            final_nexts = np.flatnonzero(next_def > next_times)
        elif len(nexts_def_idx) == 0:
            next_finish = finish_times[nexts_finish_idx[0]]
            final_nexts = np.flatnonzero(next_finish > next_times)
        else:
            next_finish = finish_times[nexts_finish_idx[0]]
            next_def = def_times[nexts_def_idx[0]]
            final_nexts = np.flatnonzero(
                np.min([next_finish, next_def]) > next_times)

        if len(final_nexts) == 0:
            continue
        final_next = next_times[final_nexts[-1]]

        if final_next <= first_next:
            continue

        games['type'].append(game_types[idx])
        games['type_time'].append(def_times[idx])
        games['start'].append(first_next)
        games['finish'].append(final_next)

        # # it is possible for a game to be loaded without ever running it.
        # if idx == len(games['start']):
        #     print('Removing game type that does not belong')
        #     games['type'] = np.delete(games['type'], idx)
        #     games['type_time'] = np.delete(games['type_time'], idx)
        #     continue
        # if games['start'][idx] < games['type_time'][idx]:
        #     # this start is not the right one. add a start at the game load time
        #     print('inserted an extra start')
        #     games['start'] = np.insert(games['start'], idx, games['type_time'])
        # # check that finish comes after the expected start
        # while ((len(games['finish']) > idx) and
        #         (games['finish'][idx] <= games['start'][idx])):
        #     games['finish'] = np.delete(games['finish'], idx)
        # if idx+1 > len(games['finish']):
        #     # we have already fixed everything else, just missing an end.
        #     # Just find the last time an action was played and call it good
        #     # enough (will cut off the last command)...
        #     print('added an end to the end')
        #     last_command_idx = np.where(
        #         hdf5_file['/game_runner/commands/data'][:] == b'next')[0][-1]
        #     last_command_time = hdf5_file['/game_runner/commands/time'][last_command_idx]
        #     games['finish'] = np.append(games['finish'], last_command_time)
        # # if this is the last item, only check previous one
        # if idx+1 == len(games['start']):
        #     continue
        # # check finish comes before the next start
        # if games['finish'][idx] > games['start'][idx+1]:
        #     games['finish'] = np.insert(
        #         games['finish'], idx, games['start'][idx+1])

    return games
