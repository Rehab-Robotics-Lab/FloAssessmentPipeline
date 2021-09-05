import h5py
from tqdm import tqdm, trange
import cv2
import math
import sys
import numpy as np

# from: https://stackoverflow.com/a/65146731/5274985

def draw_text(img, text,
              font=cv2.FONT_HERSHEY_SIMPLEX,
              pos=(0, 0),
              font_scale=1,
              font_thickness=2,
              text_color=(0, 255, 0),
              text_color_bg=(0, 0, 0),
              margin=3,
              ):

    x, y = pos
    text_size, _ = cv2.getTextSize(text, font, font_scale, font_thickness)
    text_w, text_h = text_size
    if not text_color_bg == None:
        cv2.rectangle(img, (max(pos[0]-margin, 0), max(pos[1]-margin, 0)),
                  (x + text_w+margin, y + text_h+margin), text_color_bg, -1)
    cv2.putText(img, text, (x, y + text_h + font_scale - 1),
                font, font_scale, text_color, font_thickness)

    return text_size

# from: https://www.oreilly.com/library/view/python-cookbook/0596001673/ch09s11.html
def color_scale(mag, cmin, cmax):
    """ Return a tuple of floats between 0 and 1 for R, G, and B. """
    # Normalize to 0-1
    try:
        scale = float(mag-cmin)/(cmax-cmin)
    except ZeroDivisionError:
        scale = 0.5  # cmax == cmin
    blue = min((max((4*(0.75-scale), 0.)), 1.))
    red = min((max((4*(scale-0.25), 0.)), 1.))
    green = min((max((4*math.fabs(scale-0.5)-1., 0.)), 1.))
    return int(red*255), int(green*255), int(blue*255)
