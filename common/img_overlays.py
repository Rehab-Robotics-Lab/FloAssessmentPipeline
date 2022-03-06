#!/usr/bin/env python
"""
Provides functions for drawing over images
"""
import cv2


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
        text_color_bg: background color as a tuple of (red, green, blue)
        margin: the margin around the text
    """

    x, y = pos  # pylint: disable=invalid-name
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
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
