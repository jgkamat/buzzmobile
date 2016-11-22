"""Utilities for calculating directions and distances given coords."""

import math

EARTH_RADIUS = 6.3710088e6


def get_distance(fix1, fix2):
    """Calculates great-circle distance between two positions in meters."""
    lat1 = math.radians(fix1.latitude)
    lon1 = math.radians(fix1.longitude)
    lat2 = math.radians(fix2.latitude)
    lon2 = math.radians(fix2.longitude)

    angle = (math.pow(math.sin(lat2 - lat1), 2)
             + math.cos(lat1) * math.cos(lat2)
             * math.pow(math.sin((lon2 - lon1) / 2), 2))
    unit_distance = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
    return EARTH_RADIUS * unit_distance

def get_forward_angle(fix1, fix2):
    """Calculates forward azimuth between two positions in radians."""
    lat1 = math.radians(fix1.latitude)
    lon1 = math.radians(fix1.longitude)
    lat2 = math.radians(fix2.latitude)
    lon2 = math.radians(fix2.longitude)

    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2)
               - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    angle = math.atan2(y, x)
    return (angle + 2 * math.pi) % (2 * math.pi)
