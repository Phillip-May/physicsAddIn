from robodk import robolink
from robodk import robomath

import numpy as np
import time

def print_robot_bounding_box(robot_name=None):
    # Connect to RoboDK
    RDK = robolink.Robolink()
    
    # Get the robot item
    if robot_name:
        robot = RDK.Item(robot_name, robolink.ITEM_TYPE_ROBOT)
        if not robot.Valid():
            print(f"Robot '{robot_name}' not found.")
            return
    else:
        robot = RDK.ItemUserPick('Select a robot', robolink.ITEM_TYPE_ROBOT)
        if not robot.Valid():
            print("No robot selected.")
            return

    n_links = len(robot.Joints().tolist()) + 1

    # Prompt user to select 6 objects (frames or targets) representing each link
    print("Please select 6 objects (frames or targets) representing each robot link, in order from base to tool.")
    link_objects = []
    for i in range(n_links):
        obj = RDK.ItemUserPick(f"Select object for Link {i}", robolink.ITEM_TYPE_OBJECT)
        if not obj.Valid():
            print(f"No valid object selected for Link {i}. Exiting.")
            return
        link_objects.append(obj)

    #Update the geometry pose of all links
    robotPoseAbs = robot.PoseAbs()
    for i in range(n_links):
        link = robot.ObjectLink(i)
        pose = link.PoseAbs()
        print(pose)
        poseLinkOffset = robotPoseAbs.inv() * pose
        poseLinkOffset = poseLinkOffset.inv()
        link_objects[i].setGeometryPose(poseLinkOffset)
        link_objects[i].setPoseAbs(pose)

    print("Starting update loop. Press Ctrl+C to stop.")
    try:
        while True:
            for i in range(n_links):
                link = robot.ObjectLink(i)
                pose = link.PoseAbs()
                link_objects[i].setPoseAbs(pose)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Update loop stopped.")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2:
        print("Usage: python print_robot_bounding_box.py [robot_name]")
        sys.exit(1)
    robot_name = sys.argv[1] if len(sys.argv) == 2 else None
    robot_name = 'RoboDK RDK-2200'
    print_robot_bounding_box(robot_name)