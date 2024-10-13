from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import skiros2_world_model.ros.world_model_interface as wmi

from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.abstract_skill import State

from rclpy import action

from delib_ws_problem_interface.perform_action import ACTIONS, ACTION_NAME
from pyrobosim_msgs.action import ExecuteTaskAction
from pyrobosim_msgs.msg import TaskAction, RobotState
from pyrobosim.planning.actions import ExecutionStatus

#################################################################################
# Descriptions
#################################################################################


class NavigateExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)

class PickExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

class PlaceExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

class OpenExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:OpenableLocation"), ParamTypes.Required)

class CloseExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:OpenableLocation"), ParamTypes.Required)

class GetBatteryPercentage(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("BatteryLevel", float, ParamTypes.Optional)

class BatteryAboveLevel(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("MinBatteryLevel", float, ParamTypes.Required)

class ChargerLocationFromWM(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("ChargerLocation", Element("skiros:Location"), ParamTypes.Optional)


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
        
    def onDone(self, msg) -> State:
        if msg.execution_result.status == ExecutionStatus.SUCCESS:
            return self.success("Successfully finished")
        else:
            return self.fail("Failure with msg: {}".format(msg.execution_result.message), msg.execution_result.status)

    def pyrobosimTaskAction(self, action: ACTIONS, target: str):
        """
        @brief Called when starting the skill
        @return an action msg initialized
        """
        action_goal = TaskAction()
        action_goal.robot = self.params["Robot"].value.label.split(":")[-1]
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
        self.setDescription(NavigateExecution(), "Navigate Execution")

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.NAVIGATE, self.params["TargetLocation"].value.label)

class pick_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(PickExecution(), "Pick Execution")

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.PICK, self.params["Object"].value.label)

class place_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(PlaceExecution(), "Place Execution")

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.PLACE, self.params["Object"].value.label)

class open_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(OpenExecution(), "Open Execution")

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.OPEN, self.params["Object"].value.label)
    
class close_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(CloseExecution(), "Close Execution")

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.CLOSE, self.params["Object"].value.label)

class get_battery_percentage(PrimitiveBase):
    def createDescription(self):
        self.setDescription(GetBatteryPercentage(), "Get Battery Percentage")

    def onStart(self):
        # initial ros 2 topic subscriber
        robot_name = self.params["Robot"].value.label.split(":")[-1]
        topic = "/{}/robot_state".format(robot_name)
        self.subscription = self.node.create_subscription(RobotState, topic, self.callback, 1)
        self.battery_level = None
        # Obtain world model robot element
        # self.robot = self._wmi.get_element(self.params["Robot"].value.id)
        return True
    
    def callback(self, msg):
        self.battery_level = msg.battery_level

    def execute(self):
        if self.battery_level is None:
            return self.step("Waiting for battery level")
        self.params["BatteryLevel"].value = self.battery_level
        # self.robot.setProperty("skiros:BatteryPercentage", self.battery_level)
        # self._wmi.update_element_properties(self.robot)
        return self.success("Battery level is {}".format(self.battery_level))

class battery_above_level(PrimitiveBase):
    def createDescription(self):
        self.setDescription(BatteryAboveLevel(), "Battery Above Level")

    def onStart(self):
        self.robot = self._wmi.get_element(self.params["Robot"].value.id)
        self.min_level = self.params["MinBatteryLevel"].value
        return True

    def execute(self):
        self.battery_level = self.robot.getProperty("skiros:BatteryPercentage").value
        if self.battery_level >= self.min_level:
            return self.success(f"Current battery level {self.battery_level} is above the minimum {self.min_level}.")
        else:
            return self.fail(f"Current battery level {self.battery_level} is below the minimum {self.min_level}.", -1)

class charger_location_from_wm(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ChargerLocationFromWM(), "Charger Location From World Model")

    def onStart(self):
        self.chargers = self._wmi.resolve_elements(wmi.Element(":Charger"))
        return True

    def execute(self):
        if not self.chargers:
            return self.fail("No charger found in the world model")
        self.params["ChargerLocation"].value = self.chargers[0]
        return self.success(f"Charger location set to '{self.chargers[0].id}'")
