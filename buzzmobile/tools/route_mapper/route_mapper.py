#!/usr/bin/env python

import cv2
import requests
import rospy

import datetime as dt
import googlemapskey as gmpskey
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import String


# GLOBAL VARS
route = {}
pub = rospy.Publisher('route_map', Image, queue_size=0)
bridge = CvBridge()


def get_map_url(polyline, coords):
    return ('https://maps.googleapis.com/maps/api/staticmap?size=400x400' +
            '&key=' + gmpskey.googlemapskey +
            '&path=weight:3%7Ccolor:blue%7Cenc:' + polyline +
            '&markers=icon:https://i.imgur.com/DD11yLv.png|color:blue%7Clabel:B%7C'+
            '{0},%20{1}'.format(*coords))

def get_map(polyline, coords):
    url = get_map_url(polyline, coords)
    resp = requests.get(url)
    image = cv2.imdecode(np.asarray(bytearray(resp.content), dtype='uint8'),
                         cv2.IMREAD_COLOR)
    return image

def publish_map():
    if route['polyline'] is not None and route['fix'] is not None:
        rospy.loginfo('publishing')

        route['last_published'] = dt.datetime.now()
        polyline = route['polyline'].data
        coords = route['fix'].latitude, route['fix'].longitude
        route_map = get_map(polyline, coords)
        route_msg = bridge.cv2_to_imgmsg(route_map)
        pub.publish(route_msg)

def update_polyline(new_poly):
    route['polyline'] = new_poly
    publish_map()

def update_location(new_fix):
    route['fix'] = new_fix
    if published_long_ago(): publish_map()

def published_long_ago():
    seconds_elapsed = (dt.datetime.now() - route['last_published']).total_seconds()
    return True if seconds_elapsed > 8 else False

def route_mapper_node():
    route['last_published'] = dt.datetime(2012, 10, 23)
    route['polyline'] = None
    route['fix'] = None

    rospy.init_node('route_mapper')
    rospy.Subscriber('polyline', String, update_polyline)
    rospy.Subscriber('fix', NavSatFix, update_location)
    rospy.spin()

def test():
    polyline = ('ezynEhmupUsKbGwJ_JyGkHrAbCk|@~iAm~@lhDka@``Bwe@|r@{l@tn@sb@|V}M`[kJdg@el@bs@k}@hc@o}@zaA{a@hiA{m@b_@' +
        'wo@xo@}oAhf@ut@tz@kh@pZgY|Uim@fF{lA|TsnBt]ep@kHi~CfeDyhBpzA_{@`q@ilA~xAwpBbeBkn@fx@}b@tf@ks@`k@s_@ny' +
        'Aq`@bo@_ZhcAei@~r@qc@hMchBvcA}jD`{@s~Bz|@k}@bcAkt@xxAc^vUoa@fCodAxC}eAkL}j@nQwiApt@ew@hl@c~@tu@_y@dj' +
        '@{aAlm@kw@r_Aox@xg@if@p|@ay@ln@mYvx@kv@||@{x@vIw|ArAmyAjk@{b@~i@k^tl@uPpbAeWla@}[fMakAxRag@bg@cf@d\cTrq@gj@ff@e|A`' +
        'bAkpAba@sbBrNexBaRo_BT}qAly@qrAzdCyc@zs@iLbkAiRxjCsl@by@s`@j^uU^ecA{}@wqAwfAezAb{@otAjdA_dAbiAqe@n]k' +
        'hA|tB_UxQik@~GueA`B{t@iJ{mAxi@_cBpd@wpEdaAgpDrn@}nDn}Di}Wx`ZmlTf_Vw|B`kC_wCtlEywEfqHohXn{a@_iFteGwhI' +
        '|}IqtEv_EqhAru@}nFjqDg_JfeG}_TjuNswE~wDiqF`kFutQr|QufAhsA}h@d`Amw@lvAkpB`hEyhEdqIwlB`oBetGh|FupGrtFs' +
        'mItqE_`Jn}EkzAjaAcpCjbCmoHvnG}kStmQmnBtfB{tCd`Du~LloQs`D~tEyqB~{Ak|HjrFwkKb}GkgYraQaeChsB}cAfjAcxEd}' +
        'F_vCz`DijB||AywE`nEg~EbwE_sAlpAkdEljDyaI`rEspEveCe_Fry@}mFd|@w|HdtAkpG`xAwtBlh@w_B`bAy|B~kAasDhhBgeB' +
        'xpAk|A||A{sBpaCmjBfbCmmCbkDquLnxP{dCddE_cE~aIuvHzaOaeB`gCseAlqA{Kz]fEj{Adn@f_Ije@|h@tJjn@vp@blCsSdt@' +
        'vCvqAfw@bwD~w@t{CjI`eEGbyBkDxhQs@bwDdKx~@vHxt@zDprDeQ~tCoShpAhI`lBs@xoAbHzg@ri@h{@vNfuBdC`cB^rnAgLn`' +
        '@ei@|d@af@z|@{p@nnA_{@vd@icArRigBhjAie@gU_TNmw@xd@_cAjoB_Vnu@gL|sAep@nbAkk@vrC}^fp@}WpdAmw@de@ce@dsB' +
        'mLt`AnCjj@mF~YtQt|Adj@`tGz_A|uAdsBx{BztA`rBz`@c@dCbZeJxmAeWuS')
    coords = (35.996230, -118.996203)
    cv2.imshow('map', get_map(polyline, coords))
    cv2.waitKey(0)

if __name__ == '__main__': route_mapper_node()
