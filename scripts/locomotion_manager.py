#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials');
import rospy
import smach
import smach_ros


wwsn = 'wheel_walking'
asn = '/{}/activate_locomotion'
dsn = '/{}/deactivate_locomotion'

class LocomotionMode(smach.State):
    def __init__(self, stateName):
        smach.State.__init__(
                self,
                outcomes=['SUCCESS', 'FAILURE'],
                input_keys=['request']
        )
        self.stateName = stateName
        self.activationServiceName = asn.format(stateName)
        self.deactivationServiceName = dsn.format(stateName)


    def activation_service_client():
        ## Dummy code for testing
        return True

        ## Implementation attempt
        rospy.wait_for_service(self.activationServiceName)
        try:
            activation_srv = rospy.ServiceProxy(self.activationServiceName, ActivateLocomotion)
            resp = activation_srv().msg
            return resp
        except rospy.ServiceException, e:
            print("Locomotion mode activation service call has failed: {}".format(e))
    
    
    def deactivation_service_client():
        ## Dummy code for testing
        return True

        ## Implementation attempt
        rospy.wait_for_service(self.deactivationServiceName)
        try:
            deactivation_srv = rospy.ServiceProxy(self.deactivationServiceName, DeactivateLocomotion)
            resp = deactivation_srv().msg
            return resp
        except rospy.ServiceException, e:
            print("Locomotion mode activation service call has failed: {}".format(e))

    
    def execute(self, userdata):
        if userdata.cmd == 'ACTIVATE':
            rospy.loginfo('Activating ' + self.stateName + ' logic')
            if activation_service_client():
                return self.outcomes[0]
            else:
                return self.outcomes[1]
        elif userdata.cmd == 'DEACTIVATE':
            rospy.loginfo('Deactivating ' + self.stateName + ' logic')
            if deactivation_service_client():
                rospy.loginfo('Deactivation successful for ' + self.stateName)
                return self.outcomes[0]
            else:
                rospy.logerror('Deactivation FAILED for ' + self.stateName)
                return self.outcomes[1]
        else:
            rospy.logwarn('Unknown input request for wheelwalking')


def main():
    rospy.init_node('locomotion_manager')

    sm = smach.StateMachine(outcomes=['STOPPED', 'CRASHED'])

    with sm:
        smach.StateMachine.add('WheelWalking', LocomotionMode('wheel_walking'),
            transitions={})
