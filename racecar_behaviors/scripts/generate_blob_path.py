#!/usr/bin/env python

import rospy
from astar import AStarAlgorithm
import numpy as np
from PIL import Image

class GenerateBlobPath:
    def __init__(self, start_pos,):
        self.test = 1

def main():
    rospy.init_node("generate_blob_path")
    generate_blob_path = GenerateBlobPath()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
