#!/usr/bin/env python

import rospy
import numpy as np
from PIL import Image
from nav_msgs.srv import GetMap
from astar import AStarAlgorithm
import labo_brushfire
import cv2
import rospkg
import os


class GenerateBlobPath:
    def __init__(self):
        self.map_height = None
        self.map_width = None
        self.map_res = None
        self.map_grid = None
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("racecar_behaviors")
        file_path = os.path.join(package_path, "scripts", "brushfire.bmp")
        self.astar_instance = AStarAlgorithm(file_path)
        self.get_map_attributes()
        self.get_map_grid()
        self.trajectory_counter = 0
        

    def get_map_attributes(self):
        prefix = "/racecar"
        rospy.wait_for_service(prefix + "/get_map")
        try:
            get_map = rospy.ServiceProxy(prefix + "/get_map", GetMap)
            response = get_map()
            self.map_height = response.map.info.height
            self.map_width = response.map.info.width
            self.map_res = response.map.info.resolution
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def get_map_grid(self):
        self.map_grid = labo_brushfire.grid
        # image_path = "$(find racecar_behaviors)/scripts/map.bmp"
        # image = Image.open(image_path)
        # self.map_grid = np.array(image)

    def generate_new_path_report(self, x, y):
        start_node = (0, 0)
        goal_node = self.convert_xy_coord_to_grid_cells(x, y)
        path, cost = self.astar_instance.astar(
            start_node,
            goal_node,
            self.astar_instance.m_cost,
            self.astar_instance.m_neighbors_8,
            self.astar_instance.m_h,
        )

        highlight_mask = self.map_grid == path
        self.map_grid[highlight_mask] = 40

        cv2.imwrite(
            "trajectory_object_{self.trajectory_counter}.bmp",
            cv2.transpose(cv2.flip(self.map_grid, -1)),
        )
        rospy.loginfo("Generated new blob path map")

        self.trajectory_counter += 1

    def convert_xy_coord_to_grid_cells(self, x, y):
        row = y / self.map_height
        col = x / self.map_width
        # try this if the above doesn't work
        # row = y * 0.1
        # col = x * 0.1
        return row, col


def main():
    rospy.init_node("generate_blob_path")
    generate_blob_path = GenerateBlobPath()
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
