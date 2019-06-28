#!/usr/bin/env python

import rospy
import sys
import yaml
import cv2
import os
import numpy as np
from nav_msgs.msg import OccupancyGrid
#
#
#

def write_occ_map(occ_map, content, img):
    occ_map.header.stamp = rospy.get_rostime()
    occ_map.header.frame_id = 'map'
    occ_map.info.map_load_time = occ_map.header.stamp
    occ_map.info.resolution = content['resolution']
    occ_map.info.width = img.shape[1]
    occ_map.info.height = img.shape[0]
    occ_map.info.origin.orientation.w = 1.0
    occ_map.data = [0] * (occ_map.info.width * occ_map.info.height)
    for y in range(occ_map.info.height):
        for x in range(occ_map.info.width):
            pixel_value = img[y, x]
            p = (255.0 - pixel_value) / 255.0
            i = x + (occ_map.info.height - 1 - y) * occ_map.info.width
            occ_map.data[i] = int(p * 100.0 + 0.5)
#
#
#
rospy.init_node('occupancy_server', anonymous=True)
pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10, latch=True)

if (len(sys.argv) <= 1):
    print('Please input the .yaml file of your map!')
else:
    yaml_path = sys.argv[1]
    with open(yaml_path, 'r') as f:
        content = yaml.load(f)
    yaml_dir = os.path.dirname(yaml_path)
    img_path = yaml_dir + '/' + content['image']
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if (img is None):
        print('Fail to read image!')
    else:
        print('Success to read image!')
        occ_map = OccupancyGrid()
        write_occ_map(occ_map, content, img)
        pub.publish(occ_map)
        print(min(occ_map.data))
        print(max(occ_map.data))
        print('The occupancy grid map has been published (latched)!')
        rospy.spin()
