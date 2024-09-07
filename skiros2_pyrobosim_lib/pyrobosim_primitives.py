from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase

from skiros2_std_skills.action_client_primitive import PrimitiveActionClient

from rclpy import action

from delib_ws_problem_interface.perform_action import ACTIONS, ACTION_NAME
from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import TaskAction


#################################################################################
# Descriptions
#################################################################################

class NavigateExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Target", Element("skiros:TransformationPose"), ParamTypes.Required)

class PickExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

class PlaceExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

class pyrobosim_action_client_base(PrimitiveActionClient):
    """
    @brief Base class for pyrobosim actions for the boilerplate code
    """
    def buildClient(self)->action.ActionClient:
        """
        @brief Called when starting the skill
        @return an action client
        """
        return action.ActionClient(self.node, ExecuteTaskAction, ACTION_NAME)
        

    def pyrobosimTaskAction(self, action: ACTIONS, target: str):
        """
        @brief Called when starting the skill
        @return an action msg initialized
        """
        action_goal = TaskAction()
        action_goal.robot = self.params["Robot"].value.label
        action_goal.type = action.name.lower()
        if action == ACTIONS.NAVIGATE:
            action_goal.target_location = target
        else:
            action_goal.object = target
        task_action = ExecuteTaskAction.Goal()
        task_action.action = action_goal
        return task_action

### Primitive skill implementation

class navigate_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(NavigateExecution(), self.__class__.__name__)

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.NAVIGATE, self.params["Target"].value.label)

class pick_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(PickExecution(), self.__class__.__name__)

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.PICK, self.params["Object"].value.label)

class place_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(PlaceExecution(), self.__class__.__name__)

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.PLACE, self.params["Object"].value.label)
