import numpy as np
import googlemaps
import polyline as pl
from datetime import datetime

gmaps = googlemaps.Client(key='AIzaSyC7iVrctZB49ckyfFWz8gId-snosNlBUqY')

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
    
def get_points(start, destination):
    """
    Returns the list of latitude and longitude tuples between start and 
    destination.

    start and destination may be strings or gps coordinates, e.g.,

    >>> get_directions("Sacramento, CA", "Anaheim, CA")
    [(38.58157, -121.4944), (38.58084, -121.49323), (38.58194, -121.49501), 
        ...
    (33.83344, -117.92285), (33.8354, -117.91454)]
    """
    direction_result = get_directions(start, destination)
    polyline = str(direction_result[0]['overview_polyline']['points'])
    return pl.decode(polyline)