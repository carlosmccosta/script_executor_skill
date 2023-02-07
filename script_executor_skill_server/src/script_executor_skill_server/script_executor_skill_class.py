import rospy
import actionlib
import subprocess


from script_executor_skill_msgs.msg import ScriptExecutorSkillAction, ScriptExecutorSkillResult, ScriptExecutorSkillFeedback

class ScriptExecutorSkill(object):

    def __init__(self, action_name='ScriptExecutorSkill'):
        self.action_name = action_name
        self.scripts_directory = rospy.get_param("~scripts_directory")
        self.scripts_directory_substitution_string_for_goal_script_field = rospy.get_param("~scripts_directory_substitution_string_for_goal_script_field")
        self.script_executor_skill_server = actionlib.SimpleActionServer(self.action_name, ScriptExecutorSkillAction, self.execute_skill, False)
        self.script_executor_skill_server.start()

    def execute_skill(self, goal):
        try:
            rospy.loginfo("[ScriptExecutorSkill] Received goal with script: [" + goal.script + "] and arguments [" + goal.arguments + "]")

            if goal.script:
                command = goal.script

                if self.scripts_directory_substitution_string_for_goal_script_field:
                    command = command.replace(self.scripts_directory_substitution_string_for_goal_script_field, self.scripts_directory + "/")

                if goal.arguments:
                    command = command + " " + goal.arguments

                rospy.loginfo("[ScriptExecutorSkill] Executing bash command: [" + command + "]")
                result_status = subprocess.call(command, shell=True, executable='/bin/bash')
                rospy.loginfo("[ScriptExecutorSkill] Bash command return code: [" + str(result_status) + "]")

                if result_status == 0:
                    self.success()
                else:
                    status = "ScriptExecutorSkill aborted with script return code [" + str(result_status) + "]"
                    self.aborted(status)
            else:
                self.aborted("ScriptExecutorSkill aborted due to empty script")
        except Exception as e:
            rospy.logerr('[ScriptExecutorSkill] Error: %s', str(e))
            rospy.logdebug(traceback.format_exc())

    def success(self, status=None, outcome='succeeded'):
        result_status = status if status else 'Executed goal successfully'
        result = self.result_constructor(status=result_status, outcome=outcome)
        self.script_executor_skill_server.set_succeeded(result, result.skillStatus)

    def aborted(self, status=None, outcome='aborted'):
        result_status = status if status else 'Goal aborted'
        result = self.result_constructor(status=result_status, outcome=outcome)
        self.script_executor_skill_server.set_aborted(result, result.skillStatus)

    def check_preemption(self):
        if self.script_executor_skill_server.is_preempt_requested():
            result = self.result_constructor(status='ScriptExecutorSkill Preempted', outcome='preempted')
            self.script_executor_skill_server.set_preempted(result, result.skillStatus)
            return True
        return False

    def result_constructor(self, status, outcome=''):
        result = ScriptExecutorSkillResult()
        result.skillStatus = status
        result.outcome = outcome
        self.log_info(result)
        return result

    @staticmethod
    def log_info(status):
        info = '[ScriptExecutorSkill] Status: ' + status.skillStatus
        rospy.loginfo(info)
        return info
