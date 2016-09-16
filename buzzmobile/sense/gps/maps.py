import numpy as np
import googlemaps
import polyline as pl
from datetime import datetime
from buzzmobile.sense.gps.googlemapskey import googlemapskey as gmpskey

gmaps = googlemaps.Client(key=gmpskey)

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
