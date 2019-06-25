#!/usr/bin/env python

import rospy
import time
import yaml
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid

saved_map_ = False




def callback(map):
    print('Received a %d X %d map @ %.3f m/pix' %(
        map.info.width,
        map.info.height,
        map.info.resolution))

    time_str = time.strftime("%Y-%m-%d-%H:%M:%S", time.localtime())
    map_meta_data = {'image': 'occ-map-'+time_str+'.png',
                    'resolution': 1}
    yaml_path = 'occ-map-'+time_str+'.yaml'
    with open(yaml_path, 'w') as f:
        yaml.dump(map_meta_data, f)

    img = np.zeros((map.info.height, map.info.width))

    for y in range(map.info.height):
        for x in range(map.info.width):
            i = x + (map.info.height - 1 - y) * map.info.width
            p = map.data[i] / 100.0
            pixel_value = 255.0 * (1.0 - p)
            img[y,x] = pixel_value

    img_path = 'occ-map-'+time_str+'.png'
    if (cv2.imwrite(img_path, img)):
        global saved_map_
        saved_map_ = True
        print('The occupancy grid map has been saved!')
    else:
        print('Fail to save the imgae!')




rospy.init_node('occupancy_saver', anonymous=True)
rospy.Subscriber('/grid_map_visualization/traversability_grid', OccupancyGrid, callback)
print('Waiting for occupancy grid map...')

while (not saved_map_ and not rospy.is_shutdown()):
    time.sleep(1.0)
