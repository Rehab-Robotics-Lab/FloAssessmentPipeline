#!/usr/bin/env python
"""
Provides functions for generating color values
"""
import math


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
