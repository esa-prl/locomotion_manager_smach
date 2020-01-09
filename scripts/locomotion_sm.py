#!/usr/bin/env python


"""
TODOs:
- Use the smach.ServiceState for subclassing in Reset and Locomotion (not Guard)
- Check if transition is allowed when receiving request via service before addinging it to the user data.
- Have elegant way to manually and automatically define modes
- rename reset node to init or similar
"""

#import roslib; roslib.load_manifest('smach_tutorials');
from functools import partial
import copy

import rospy
import smach
import smach_ros
from locomotion_manager.srv import *

# Consider feeding them as configurations / launch parameters
mode_names = ['wheel_walking', 'crabkerman']
all_mode_names = copy.copy(mode_names)
all_mode_names.append('leveling')

default_mode_name = mode_names[0]
asn = '/{}/activate_locomotion'
dsn = '/{}/deactivate_locomotion'

# These are internal for the smach, no need to make them visible outside
rst_name = 'init'

def locomotionSelectionCallback(req, stateMachine):
    rospy.loginfo('>>> Locomotion selection request: ' + req.request)

    if req.request not in all_mode_names:
        rospy.logwarn('Received illegal locomotion mode request: {}'.format(req.request))
        return LocomotionSelectionResponse(False)

    active_state_name = stateMachine.get_active_states()[0]
    active_state = stateMachine.__getitem__(active_state_name)

    registered_outcomes = active_state.get_registered_outcomes()

    if 'GOTO_'+req.request in registered_outcomes:

        # Pass request to currently active state.
        stateMachine.userdata.sm_target_state = req.request
    else:
        rospy.logwarn('Transition from {} to {} not possible!'.format(active_state_name, req.request))
        return LocomotionSelectionResponse(False)

    return LocomotionSelectionResponse(True)

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
    def __init__(self, stateName, outcomes, rate=10):
        self.outcomes = outcomes
        smach.State.__init__(
            self,
            outcomes=self.outcomes,
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

    sm = smach.StateMachine(outcomes=['CRASHED'])

    with sm:
        loc_trans = {'GOTO_'+name : name for name in mode_names}
        loc_trans['FAILURE'] = 'CRASHED'
        rst_trans = copy.copy(loc_trans)



        
        # Reset state logic
        sm.add(rst_name, ResetState(),
            rst_trans, remapping = {
                'target_state' : 'sm_target_state',
            }
        )

        # Mode transition logic
        for name in mode_names: 
            mode_trans = copy.copy(loc_trans)
            del mode_trans['GOTO_'+name]
            # del mode_trans['GOTO_leveling']

            if name == 'wheel_walking':
                mode_trans['GOTO_leveling'] = 'leveling'

                print(mode_trans.keys())

            sm.add(name, LocomotionMode(name, outcomes=mode_trans.keys(), rate=1),

                transitions = mode_trans
                , remapping = {
                    'target_state' : 'sm_target_state',
                }
            )
    
        lvl_outcomes = ['GOTO_wheel_walking', 'FAILURE']
        sm.add('leveling', LocomotionMode('leveling', outcomes=lvl_outcomes, rate=1),
            transitions = {'GOTO_wheel_walking' : 'wheel_walking',
                            'FAILURE' : 'CRASHED'}
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

    # Start visual server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/LOC_SM')
    sis.start()

    # Start state machine
    internalExecution(sm, default_mode_name)

    rospy.spin()
    sis.stop()

    rospy.loginfo('Locomotioin manager shutting down...')


if __name__ == '__main__':
    main()

