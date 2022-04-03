import matplotlib.pyplot as plt


def plot_model_results(times, raw_data, model_output, start_idx=0, num_idx=None, title=None):
    if num_idx is None:
        num_idx = len(times)-start_idx
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 5), dpi=200)
    ax1.plot(times[start_idx:start_idx+num_idx],
             raw_data[start_idx:start_idx+num_idx, 0:3], linewidth=1, alpha=0.3)
    ax1.plot(times[start_idx:start_idx+num_idx],
             model_output[start_idx:start_idx+num_idx, 0:3], linewidth=.5, alpha=.8)
    ax1.legend(['x-raw', 'y-raw', 'z-raw', 'x', 'y', 'z'])
    ax2.plot(times[start_idx:start_idx+num_idx],
             model_output[start_idx:start_idx+num_idx, 3:6], linewidth=0.5, alpha=.8)
    ax2.legend(['dx', 'dy', 'dz'])
    ax3.plot(times[start_idx:start_idx+num_idx],
             model_output[start_idx:start_idx+num_idx, 6:9], linewidth=0.5, alpha=.8)
    ax3.legend(['ddx', 'ddy', 'ddz'])
    if title:
        fig.suptitle(title)
