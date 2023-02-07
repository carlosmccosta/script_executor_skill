#!/usr/bin/python3

import rospy
import traceback
from script_executor_skill_server.script_executor_skill_class import ScriptExecutorSkill


if __name__ == "__main__":

    rospy.init_node('script_executor_skill')

    try:
        actionName = rospy.get_param('~action_name')
    except Exception as e:
        raise KeyError('Unable to access ROS parameter server for ' + str(actionName))

    try:
        ScriptExecutorSkill = ScriptExecutorSkill(actionName)
        rospy.spin()

    except Exception as e:
        rospy.logerr('[ScriptExecutorSkill] Error: %s', str(e))
        rospy.logdebug(traceback.format_exc())
        quit()
