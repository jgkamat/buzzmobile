import xml.etree.ElementTree as ET
from gzb_to_latlon import convert

def straight_road_coordinates_to_latlons(road_coordinates):
    latlons = []
    for coordinate in road_coordinates:
        (lat, lon) = convert(coordinate[0], coordinate[1])
        latlons.append((lat, lon))
    return latlons
