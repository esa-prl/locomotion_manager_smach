#!/usr/bin/env python


"""
TODOs:
+ Semantic:
    - Determine what happens when calling the state we're already in
    - Determine HALT behavior
+ Integration:
    - Determine naming conventions and extract names to params or launch args
+ Internal implementation:
    - Use the smach.ServiceState for subclassing in Reset and Locomotion (not Guard)
"""

#import roslib; roslib.load_manifest('smach_tutorials');
from functools import partial
import copy

import rospy
import smach
import smach_ros
from locomotion_manager.srv import *

# Consider feeding them as configurations / launch parameters
mode_names = ['simple','wheel_walking', 'crabkerman']
default_mode_name = mode_names[0]
asn = '/{}/activate_locomotion'
dsn = '/{}/deactivate_locomotion'

# These are internal for the smach, no need to make them visible outside
guard_name = 'guard'
rst_name = 'reset'

def locomotionSelectionCallback(req, stateMachine):
    rospy.loginfo('>>> Locomotion selection request: ' + req.request)

    if req.request not in mode_names:
        rospy.logwarn('Received illegal locomotion mode request: {}'.format(req.request))
        return LocomotionSelectionResponse(False)

    # Pass request to currently active state.

    # stateMachine.userdata.sm_current_state = stateMachine.userdata.sm_target_state
    stateMachine.userdata.sm_target_state = req.request
    # stateMachine.execute()
    return LocomotionSelectionResponse(True)


class GuardState(smach.State):
    def __init__(self):
        outcomes = ['GOTO_' + name for name in mode_names]
        # CAREFUL: should RUNNING be added?
        smach.State.__init__(
            self,
            outcomes=outcomes + ['RUNNING'],
            input_keys=['current_state', 'target_state'],
            output_keys=['current_state', 'target_state']
        )
    
    def execute(self, userdata):
        if userdata.current_state == 'INIT':
            userdata.current_state = userdata.target_state
            return 'GOTO_' + userdata.target_state
        # Is this the behavior we're looking for when invoking a state
        # we're already in? CHANGE ABOVE IF YOU CHANGE THIS!
        elif userdata.current_state == userdata.target_state:
            return 'RUNNING'
        return 'GOTO_' + userdata.current_state

# class LocState(smach.State):
#   def __init(self):
#       outcomes =  = \
#           ['FAILURE'] + \
#           [name for name in mode_names]
#       smach.State.__init__(
#           self,
#           outcomes=outcomes,
#           input_keys=['current_state', 'target_state'],
#           output_keys=['current_state', 'target_state']
#       )

# Should probably subclass smach.ServiceState in a future iteration
class ResetState(smach.State):
    def __init__(self):
        outcomes = \
            ['FAILURE'] + \
            ['GOTO_' + name for name in mode_names]
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=['target_state'],
            output_keys=['target_state']
        )

    def reset_service_client(self):
        return True
    
    def execute(self, userdata):
        self.reset_service_client()
        rospy.loginfo('Reset service call successful')
        return 'GOTO_' + userdata.target_state


# State subclass for switching locomotion modes
# Should probably subclass smach.ServiceState in a future iteration
class LocomotionMode(smach.State):
    def __init__(self, stateName, rate=10):
        outcomes = \
            ['FAILURE'] + \
            ['GOTO_' + name for name in mode_names]
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=['target_state'],
            output_keys=['target_state']
        )

        self.rate = rospy.Rate(rate)

        self.stateName = stateName
        self.activationServiceName = asn.format(stateName)
        self.deactivationServiceName = dsn.format(stateName)


    def activation_service_client(self):
        ## Dummy code for testing
        return True

        ## Implementation attempt
        rospy.wait_for_service(self.activationServiceName)
        try:
            activation_srv = rospy.ServiceProxy(self.activationServiceName, ActivateLocomotion)
            resp = activation_srv().msg
            return resp
        except rospy.ServiceException as e:
            rospy.logerror("Locomotion mode activation service call has failed: {}".format(e))
    
    
    def deactivation_service_client(self):
        ## Dummy code for testing
        return True

        ## Implementation attempt
        rospy.wait_for_service(self.deactivationServiceName)
        try:
            deactivation_srv = rospy.ServiceProxy(self.deactivationServiceName, DeactivateLocomotion)
            resp = deactivation_srv().msg
            return resp
        except rospy.ServiceException as e:
            rospy.logerror("Locomotion mode activation service call has failed: {}".format(e))

    
    def execute(self, userdata):
        rospy.loginfo('Activating ' + self.stateName + ' logic')
        if self.activation_service_client():
            rospy.loginfo('Activated  ' + self.stateName + ' logic')
        else:
            rospy.logerror('Activation FAILED for ' + self.stateName)
            return 'FAILURE'

        rospy.loginfo('Waiting for new state...')
        while userdata.target_state == self.stateName and not rospy.is_shutdown():
            self.rate.sleep()

        rospy.loginfo('Deactivating ' + self.stateName + ' logic')
        if self.deactivation_service_client():
            rospy.loginfo('Deactivation successful for ' + self.stateName)
            return 'GOTO_' + userdata.target_state
        else:
            rospy.logerror('Deactivation FAILED for ' + self.stateName)
            return 'FAILURE'



def internalExecution(stateMachine, targetMode):
    stateMachine.userdata.sm_current_state = stateMachine.userdata.sm_target_state
    stateMachine.userdata.sm_target_state = targetMode
    stateMachine.execute()


def main():
    rospy.init_node('locomotion_manager')

    sm = smach.StateMachine(outcomes=['DRIVING', 'HALTED', 'CRASHED'])

    with sm:
        guard_trans = {'GOTO_'+name : name for name in mode_names}
        rst_trans = copy.copy(guard_trans)
        loc_trans = copy.copy(guard_trans)
        loc_trans.update({'FAILURE': 'CRASHED'})


        # Guard state logic
        guard_trans['RUNNING'] = 'DRIVING'  # is this needed?
        sm.add(guard_name, GuardState(),
            transitions = guard_trans, remapping = {
                'current_state' : 'sm_current_state',
                'target_state' : 'sm_target_state',
            }
        )
        
        # Reset state logic
        rst_trans['FAILURE'] = 'CRASHED'
        sm.add(rst_name, ResetState(),
            rst_trans, remapping = {
                'target_state' : 'sm_target_state',
            }
        )

        # Mode transition logic
        for name in mode_names:
            sm.add(name, LocomotionMode(name, rate=1),

            transitions = loc_trans
                , remapping = {
                    'target_state' : 'sm_target_state',
                }
            )
    
    sm.userdata.sm_target_state = 'INIT'

    ### DEBUGGING ONLY ###
    """
    internalExecution(sm, mode_names[1])
    internalExecution(sm, mode_names[0])
    internalExecution(sm, mode_names[0])
    """
    ######################


    # Pass the state machine to the service callback with a partial function
    locomSelCb = partial(locomotionSelectionCallback, stateMachine=sm)
    srv = rospy.Service('locomotion_selection', LocomotionSelection, locomSelCb)
    rospy.loginfo('Locomotion selection service is now online!')
    internalExecution(sm, default_mode_name)

    rospy.spin()

    rospy.loginfo('Locomotioin manager shutting down...')


if __name__ == '__main__':
    main()

