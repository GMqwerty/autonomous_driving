#!/usr/bin/env python

import roslaunch
import rospy

# rospy.init_node("tester")

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

launch_file = ['carpkg', 'run2.launch']
roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(launch_file)

parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

parent.start()

try:
  parent.spin()
finally:
  parent.shutdown()