#!/usr/bin/env python
import roslib#; roslib.load_manifest('smach_usecase')

import rospy
import smach
import smach_ros
import std_srvs.srv #.srv
import ail_mav.srv
import actionlib
import mav_state_machine.msg

class Landed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['takeoff_started'])
        self.counter=0
    def execute(self,userdata):
        rospy.loginfo('Landed status')
        rospy.sleep(3.0)
        return 'takeoff_started'


def main():
    rospy.init_node('drone_executive')
    sm_root = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    with sm_root:
        # CONNECTING is the initial state, 
        # if connected, goto LANDED state
        @smach.cb_interface(outcomes=['aborted'])
        def connect_resp_cb(userdata,response):
            if response.success == False:
                return 'aborted'
            else:
                return 'succeeded'
        req_connect = ail_mav.srv.ConnectMavRequest('/mavros', True)
        smach.StateMachine.add('CONNECTING',
            smach_ros.ServiceState('/mav_interface_node/connect',
                ail_mav.srv.ConnectMav,req_connect,
                response_cb=connect_resp_cb),
            transitions={'succeeded':'LANDED','aborted':'CONNECTING'})

        # Come to LANDED after CONNECTING or LANDING
        # as defined in class Landed, hold 3 seconds, goto TAKINGOFF
        smach.StateMachine.add('LANDED',Landed(),
            transitions={'takeoff_started':'TAKINGOFF'})

        # Come to TAKINGOFF after time of LANDED expires
        # if responded success, goto
        # if aborted, goto LANDING
        @smach.cb_interface(outcomes=['aborted'])
        def takeoff_resp_cb(userdata,response):
            if response.success == False:
                return 'aborted'
            else:
                return 'succeeded'
        req_takeoff = ail_mav.srv.TakeoffRequest(1.0)
        smach.StateMachine.add('TAKINGOFF',
            smach_ros.ServiceState('/mav_interface_node/takeoff', 
                ail_mav.srv.Takeoff,req_takeoff,
                response_cb=takeoff_resp_cb),
            transitions={'succeeded':'FLYING_TO_WAITING_ZONE','aborted':'LANDING'})

        # Come to LANDING after TAKINGOFF aborted or FLYING_TO_LAND
        # goto LANDED afterwards
        smach.StateMachine.add('LANDING',
            smach_ros.ServiceState('/mav_interface_node/land', 
                ail_mav.srv.Land),#,ail_mav.srv.LandRequest.Empty),
            transitions={'succeeded':'LANDED'})

        #FLYING_TO_WAITING_ZONE
        goal = mav_state_machine.msg.trackingGoal() 
        goal.orientation_final = 0.0
        goal.pos_final.x = -1.0
        goal.pos_final.y = -0.4
        goal.pos_final.z = 1.8
        smach.StateMachine.add('FLYING_TO_WAITING_ZONE',
            smach_ros.SimpleActionState('/traj_plan_track_action_server',
                mav_state_machine.msg.trackingAction,
                goal
                ),
            transitions={'succeeded':'FLYING_TO_LAND'})

        goal2 = mav_state_machine.msg.trackingGoal() 
        goal2.orientation_final = 0.0
        goal2.pos_final.x = 0.0
        goal2.pos_final.y = 0.0
        goal2.pos_final.z = 1.0
        smach.StateMachine.add('FLYING_TO_LAND',
            smach_ros.SimpleActionState('/traj_plan_track_action_server',
                mav_state_machine.msg.trackingAction,
                goal2
                ),
            transitions={'succeeded':'LANDING'})

    sis = smach_ros.IntrospectionServer('intro_server', sm_root, '/SM_ROOT')
    sis.start()
    outcome = sm_root.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
