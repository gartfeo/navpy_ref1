import math

import numpy as np
import pymap3d

# Placeholder constants for values like gravity, which are not defined in the snippet
M_PI = 3.141592653589793238462643383279502884
GRAVITY_MSS = 9.80665
LATLON_TO_M = 111318.84502145034

#     // scaling factor degrees to meters at equator
#     // == DEG_TO_RAD * RADIUS_OF_EARTH
LOCATION_SCALING_FACTOR = LATLON_TO_M


def wrap_360_cd(angle):
    res = angle % 36000
    if res < 0:
        res += 36000
    return res


def wrap_180_cd(angle):
    res = wrap_360_cd(angle)
    if res > 18000:
        return res - 36000
    return res


def wrap_2PI(radian):
    res = math.fmod(radian, 2 * np.pi)
    if res < 0:
        res += 2 * np.pi
    return res


def wrap_PI(radian):
    res = wrap_2PI(radian)
    if res > np.pi:
        return res - 2 * np.pi
    return res


def longitude_scale(lat):
    scale = np.cos(lat * (np.pi / 180.0))
    return max(scale, 0.01)


def diff_longitude(lon1, lon2):
    # get lon1-lon2, wrapping at -180e7 to 180e7
    if np.sign(lon1) == np.sign(lon2):
        # common case of same sign
        return lon1 - lon2
    dlon = int(lon1) - int(lon2)
    if dlon > 180:
        dlon -= 360
    elif dlon < -180:
        dlon += 360
    return dlon


def get_distance_NE(loc, loc2):
    ned = pymap3d.geodetic2ned(loc2.lat, loc2.lng, loc2.alt, loc.lat, loc.lng, loc.alt)
    # [(loc2.lat - loc.lat) * LOCATION_SCALING_FACTOR,
    #  diff_longitude(loc2.lng, loc.lng) * LOCATION_SCALING_FACTOR * longitude_scale((loc2.lat + loc.lat) / 2)]
    return [ned[0], ned[1]]


def get_distance_ND(loc, loc2):
    ned = pymap3d.geodetic2ned(loc2.lat, loc2.lng, loc2.alt, loc.lat, loc.lng, loc.alt)
    ne_proj = np.linalg.norm([ned[0], ned[1]])
    return [ne_proj, ned[2]]

# DEGX100 = 5729.57795
# def get_bearing(loc, loc2):
#     off_x = diff_longitude(loc2.lng, loc.lng)
#     off_y = (loc2.lat - loc.lat) / longitude_scale((loc.lat + loc2.lat) / 2)
#     bearing = (np.pi * 0.5) + np.arctan2(-off_y, off_x)
#     if bearing < 0:
#         bearing += 2 * np.pi
#     return bearing
#
#
# def get_bearing_to(loc, loc2):
#     return int(get_bearing(loc, loc2) * DEGX100 + 0.5)
