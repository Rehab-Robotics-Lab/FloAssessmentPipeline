"""Tools for plotting results"""

import matplotlib.pyplot as plt
import numpy as np


def plot_model_results(times, raw_data, model_output, start_idx=0, num_idx=None, title=None):
    """Plot a 3 row subplot with x,y,z; dx,dy,dz; ddx,ddy,ddz. Plots raw values for x,y,z
    in background.

    Shifts time to start at 0.

    Args:
        times: The times that data was captured at.
        raw_data: The raw x,y,z data
        model_output: The state produced by the kalman filter, etc.
        start_idx: The start frame
        num_idx: The number of frames to plot
        title: The title of the plot
    """
    # pylint: disable=too-many-arguments
    legend_label_spacing = 0
    # colors = plt.get_cmap('viridis')(np.linspace(0, 1, 5))[1:4]
    colors = ["#274569", "#AE629F", "#D8BA96"]
    if num_idx is None:
        num_idx = len(times)-start_idx
    if model_output.shape[1] == 9:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(
            5.4, 4), dpi=100, sharex=True, sharey=False)
    elif model_output.shape[1] == 6:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(
            5.4, 8/3), sharex=True, sharey=False, dpi=100)

    fig.tight_layout(rect=(0.0, 0.03, 1, 1))  # ,h_pad=3)
    ax1.set_prop_cycle('color', colors)
    ax1.plot(times[start_idx:start_idx+num_idx]-times[start_idx],
             raw_data[start_idx:start_idx+num_idx, 0:3], linewidth=2, alpha=0.4)
    ax1.set_prop_cycle('color', colors)
    ax1.plot(times[start_idx:start_idx+num_idx]-times[start_idx],
             model_output[start_idx:start_idx+num_idx, 0:3], linewidth=.5, alpha=1)
    ax1.legend(['$x$ raw', '$y$ raw', '$z$ raw', '$x$', '$y$', '$z$'],
               loc='right', labelspacing=legend_label_spacing)
    ax1.set_ylabel('Position\n(mm)')
    ax2.set_prop_cycle('color', colors)
    ax2.plot(times[start_idx:start_idx+num_idx]-times[start_idx],
             model_output[start_idx:start_idx+num_idx, 3:6], linewidth=0.5, alpha=1)
    ax2.legend(['$\dot{x}$', '$\dot{y}$', '$\dot{z}$'], loc='right',
               labelspacing=legend_label_spacing)
    ax2.set_ylabel('Velocity\n(mm/s)')
    if model_output.shape[1] == 9:
        ax3.set_prop_cycle('color', colors)
        ax3.plot(times[start_idx:start_idx+num_idx]-times[start_idx],
                 model_output[start_idx:start_idx+num_idx, 6:9], linewidth=0.5, alpha=1)
        ax3.legend(['$\ddot{x}$', '$\ddot{y}$', '$\ddot{z}$'], loc='right',
                   labelspacing=legend_label_spacing)
        ax3.set_ylabel('Acceleration\n(mm/s$^2$)')
    if title:
        fig.suptitle(title)
    fig.supxlabel('Time (s)')
