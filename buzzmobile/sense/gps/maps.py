import numpy as np
import googlemaps
import polyline as pl
from datetime import datetime
import math

#from buzzmobile.sense.gps.googlemapskey import googlemapskey as gmpskey
import googlemapskey as gmpskey

gmaps = googlemaps.Client(key=gmpskey.googlemapskey)

def get_directions(start, destination):
    """
    Returns a list of current driving directions from start to destination.
    start and destination may be strings or gps coordinates, e.g.,

    >>> get_directions("Atlanta, GA", "Seattle, WA")
    [direction object 1, direction object 2, ...]

    >>> get_directions((20.123, 41.382), (42.000, 3.142))
    [direction object 1, direction object 2, ...]
    """
    now = datetime.now()
    direction_result = gmaps.directions(origin = start,
                                        destination = destination,
                                        alternatives = True,
                                        mode="driving",
                                        units="metric",
                                        departure_time=now)
    return direction_result

def get_points(start, destination, top_left=None, width_height=None):
    """
    Returns the list of latitude and longitude tuples between start and
    destination within a "rectangle" specified by a top left corner lat-long
    tuple and a width/height tuple.

    start and destination may be strings or gps coordinates, e.g.,

    >>> get_directions("Sacramento, CA", "Anaheim, CA")
    [(38.58157, -121.4944), (38.58084, -121.49323), (38.58194, -121.49501),
        ...
    (33.83344, -117.92285), (33.8354, -117.91454)]

    >>> get_directions("Sacramento, CA", "Atlanta, GA", (34, -88.6), (0.3, 0.3))
    [(34.27317, -88.50898), (34.23178, -88.42559), (34.23086, -88.32019)]
    """
    #get the directions
    direction_result = get_directions(start, destination)
    polyline = str(direction_result[0]['overview_polyline']['points'])

    #get the list of lat-long coordinates from the directions
    points = pl.decode(polyline)

    #return only the points in the specified range
    if top_left is None or width_height is None:
        return points
    else:
        return get_points_in_rect(points, top_left, width_height)

def get_points_in_rect(points, top_left, width_height):
    """
    Returns a list of point tuples within a rectangle specified by a top_left 
    corner and a width_height tuple.

    >>> get_points_in_rect([(0, 0), (2, 2), (3, 4), (5, -1)], (0, 2), (2, 2))
    [(2, 2)]
    """
    #calculate the bottom right corner to use repeatedly for range checking
    bottom_right = (top_left[0] + width_height[0], top_left[1] + width_height[1])
    #return the points in the specified range
    return [i for i in points 
            if i[0] <= bottom_right[0] and i[1] <= bottom_right[1] 
            and i[0] >= top_left[0] and i[1] >= top_left[1]]
            
def haversine(lat1, lon1, lat2, lon2):
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    a = math.sin(delta_lat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(delta_lon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    R = 6371e3
    d = R * c
    return d
