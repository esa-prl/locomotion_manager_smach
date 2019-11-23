#!/usr/bin/env python

import sys
import rospy
from locomotion_manager.srv import LocomotionSelection

def loc_sel_client(locmode):
    rospy.wait_for_service('locomotion_selection')
    try:
        ls = rospy.ServiceProxy('locomotion_selection', LocomotionSelection)
        resp1 = ls(locmode)
        return "--- IT WORKED ---"
    except rospy.ServiceException, e:
        rospy.loginfo("Service call failed: {}".format(str(e)))

def usage():
    return "%s <LOC_MODE_NAME>"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        locname = sys.argv[1]
    else:
        print(usage())
        sys.exit(1)
    rospy.loginfo("Requesting {}".format(locname))
    rospy.loginfo("Response was: {}".format(loc_sel_client(locname)))
