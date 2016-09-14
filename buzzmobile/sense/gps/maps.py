import numpy as np
import googlemaps
import polyline as pl
from datetime import datetime

gmaps = googlemaps.Client(key='AIzaSyC7iVrctZB49ckyfFWz8gId-snosNlBUqY')

def get_directions(start, destination):
    now = datetime.now()
    direction_result = gmaps.directions(origin = start,
                                        destination = destination,
                                        alternatives = True,
                                        mode="driving",
                                        units="metric",
                                        departure_time=now)
    return direction_result 
    
def get_points(direction_result):
    with open("path.txt", "r+") as f:
        f.write(str(direction_result))
    polyline = str(direction_result[0]['overview_polyline']['points'])
    return pl.decode(polyline)