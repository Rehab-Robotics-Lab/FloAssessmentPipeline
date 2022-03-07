#!/usr/bin/env python
"""
Provides functions for drawing over images
"""
import cv2
import numpy as np


def draw_text(img, text,  # pylint: disable=too-many-arguments
              font=cv2.FONT_HERSHEY_SIMPLEX,
              pos=(0, 0),
              font_scale=1,
              font_thickness=2,
              text_color=(0, 255, 0),
              text_color_bg=(0, 0, 0),
              margin=3):
    """Draw text on an image using opencv

    Args:
        img: opencv compatible image
        text: text to write
        font: the font to use (find here:
              https://docs.opencv.org/master/d6/d6e/group__imgproc__draw.html#ga0f9314ea6e35f99bb23f29567fc16e11)
        pos: the position to start drawing at
        font_scale: scale to draw
        font_thickness: thickness of the lines in text
        text_color: text color as a tuple of (red, green, blue)
        text_color_bg: background color as a tuple of (red, green, blue). Pass in None to suppress
        margin: the margin around the text
    """

    x, y = pos  # pylint: disable=invalid-name
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    if text_color_bg:
        cv2.rectangle(img, (max(pos[0]-margin, 0), max(pos[1]-margin, 0)),
                      (x + text_w+margin, y + text_h+margin), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + font_scale - 1),
                font, font_scale, text_color, font_thickness)

    return text_size


def draw_cam_info(img, idx=None, time=None, cam=None, cam_type="color"):
    """Draw camera info on frame.

    index, time, and cam arguments are optional,
    if not passed, they will not be drawn.

    will edit the image directly

    Args:
        img: numpy/cv2 image
        idx: The frame index
        time: The frame time
        cam: The camera name for the frame
        cam_type: the type of camera
    """
    if idx:
        draw_text(img, f'frame: {idx}', pos=(100, 3))
    if time:
        draw_text(img, f'time: {time:.2f}', pos=(500, 3))
    if cam:
        draw_text(img, f'view: {cam} realsense {cam_type}', pos=(900, 3))


class DataPlot:
    """Class to create data plots which scroll, kind of like an osciliscope
    display.
    """

    def __init__(self, pos, size, label=None, domain_length=15,
                 buffer=500, color=(250, 250, 250), thickness=4,
                 border=1, blur=None):
        """Create dataplot object

        Args:
            pos: position to place plot (x,y)
            size: size to make plot (width, height)
            label: Label to print for plot, None to supress
            domain_length: the amount of the domain (amount of
                           x-data) to plot. Plots the newest
                           domain_length quantity of data
            buffer: how much data to buffer. Make this >
                    domain_length*fps
            color: The color for the plot
            thickness: The plot thickness
            border: Border thickness, None to supress
            blur: The amount of gaussian blur to add under
                  the plot area (this is the kernel size,
                  used to calculate st dev of the kernel).
                  None to supress
        """
        #pylint: disable=too-many-arguments
        self.x_vals = np.zeros(buffer, dtype=np.float64)
        self.y_vals = np.zeros(buffer, dtype=np.float64)
        self.first = True
        self.domain_length = domain_length
        self.pos = pos
        self.size = size
        # prevent division by zero if no data yet exists:
        self.max_y = .0001
        self.min_y = -.0001
        self.label = label
        self.color = color
        self.thickness = thickness
        self.border = border
        self.blur = blur

    def add_data(self, x_val, y_val):
        """Add data to the plot

        This is done seperately from plotting, add data as you want to,
        plot when it makes sense

        Args:
            x_val: the x value to add
            y_val: the y value to add
        """
        self.x_vals = np.roll(self.x_vals, 1)
        self.y_vals = np.roll(self.y_vals, 1)
        self.x_vals[0] = x_val
        self.y_vals[0] = y_val
        self.max_y = max(self.max_y, y_val)
        self.min_y = min(self.min_y, y_val)
        if self.first:
            self.x_vals[-1] = x_val
            self.y_vals[-1] = y_val
            self.first = False
            self.max_y = y_val + 0.0001
            self.min_y = y_val - 0.0001

    def plot(self, img):
        """Plot the stored data onto an image

        Args:
            img: An image to plot onto
        """
        adj_x_vals = self.x_vals - self.x_vals[0]
        in_range = adj_x_vals > (-1*self.domain_length)
        scaled_x_vals = (adj_x_vals / self.domain_length + 1)*self.size[0]
        # convert data to pixels
        scaled_y_vals = self.size[1]*(
            (self.y_vals - self.min_y)/(self.max_y - self.min_y))
        if self.blur:
            kernel_size = self.blur
            left_side = max(0, self.pos[0]-kernel_size)
            right_side = min(
                img.shape[0], self.pos[0]+self.size[0]+kernel_size)
            top_side = max(0, self.pos[1]-kernel_size)
            bottom_side = min(
                img.shape[1], self.pos[1]+self.size[1]+kernel_size)

            img[self.pos[1]:(self.pos[1]+self.size[1]),
                self.pos[0]:(self.pos[0]+self.size[0]), :] = \
                cv2.GaussianBlur(
                img[top_side:bottom_side, left_side:right_side, :], (kernel_size, kernel_size), 0)[
                (self.pos[1]-top_side):(self.pos[1]+self.size[1]-bottom_side),
                (self.pos[0]-left_side):(self.pos[0]+self.size[0]-right_side),
                :]
        cv2.polylines(
            img,
            [np.int32(
                np.transpose(
                    (self.pos[0] + scaled_x_vals[in_range],
                     self.pos[1] + (scaled_y_vals[in_range]))))],
            isClosed=False, color=self.color, thickness=self.thickness)
        if self.border:
            cv2.polylines(img, [np.array([
                [self.pos[0], self.pos[1]],
                [self.pos[0], self.pos[1] + self.size[1]],
                [self.pos[0]+self.size[0], self.pos[1] + self.size[1]],
                [self.pos[0]+self.size[0], self.pos[1]],
                [self.pos[0], self.pos[1]]
            ], np.int32)],
                isClosed=False, color=self.color, thickness=1)
        if self.label:
            draw_text(img, self.label, pos=(self.pos[0]+3, self.pos[1]+3),
                      text_color_bg=None, text_color=self.color)
