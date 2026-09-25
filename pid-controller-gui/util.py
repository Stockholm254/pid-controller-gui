"""
util.py - utility things for all other modules


function resource_path
    routine to correct a given path to some resource in accordance to whether the program is running in frozen mode or
    not (see https://stackoverflow.com/questions/7674790/bundling-data-files-with-pyinstaller-onefile)

function fit_line
    least-squares straight line through (time, value) samples - gives a smooth derivative of noisy data
"""

import os
import sys


def resource_path(relative_path: str) -> str:
    """
    Use it to correct a given path to some resource in accordance to whether the program is running in frozen mode or
    not. Wrap an every request for a local file by this function to be able to run the script both in normal and bundle
    mode without any changes (see https://stackoverflow.com/questions/7674790)

    :param relative_path: string representing some relative path
    :return: appropriate path
    """

    if hasattr(sys, '_MEIPASS'):
        if relative_path[:2] == '..':
            # remove parent directory identifier
            return os.path.join(sys._MEIPASS, relative_path[3:])
        else:
            return os.path.join(sys._MEIPASS, relative_path)

    return relative_path


def fit_line(samples) -> tuple:
    """
    Least-squares straight line through (time, value) samples. Its slope is a smooth derivative of noisy data and its
    value at the newest time is a smoothed current value

    :param samples: iterable of (time in seconds, value) pairs
    :return: (slope per second, fitted value at the newest time) or None if there are less than 3 samples or all of them
    have the same time
    """

    samples = list(samples)
    if len(samples) < 3:
        return None

    t_mean = sum(t for t, _ in samples) / len(samples)
    v_mean = sum(v for _, v in samples) / len(samples)
    s_tt = sum((t - t_mean) ** 2 for t, _ in samples)
    if s_tt == 0.0:
        return None
    s_tv = sum((t - t_mean) * (v - v_mean) for t, v in samples)

    slope = s_tv / s_tt
    t_newest = max(t for t, _ in samples)
    return slope, v_mean + slope * (t_newest - t_mean)
