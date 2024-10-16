from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, Selector, Serial, RetryOnFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element

import skiros2_world_model.ros.world_model_interface as wmi

from .pyrobosim_compound_skills import Navigate, Pick, Place, OpenOpenableLocation, CloseOpenableLocation


# Note: Feel free to write your own skills and primitives here.
#       You can add the skill implementation names like "my_primitive" and "my_skill" to the list of skills to load in "main.launch.py"


#################################################################################
# Descriptions
#################################################################################

class MyPrimitive(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("WorldModelObject", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("WorldModelOptional", Element("skiros:Location"), ParamTypes.Optional)
        self.addParam("DictionaryOptional", dict, ParamTypes.Optional)
        self.addParam("Boolean", False, ParamTypes.Required)
        self.addParam("Number", 0.0, ParamTypes.Required)

class MySkill(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("WorldModelObject", Element("skiros:Location"), ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

class my_primitive(PrimitiveBase):
    """
    This primitive has 3 states
    """
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(MyPrimitive(), self.__class__.__name__)

    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.fail("Stopped", -1)

    def onStart(self):
        """Called just before 1st execute"""
        return True

    def execute(self):
        """ Main execution function. Should return with either: self.fail, self.step or self.success."""
        if self._progress_code < 10:
            return self.step("Step")
        else:
            return self.success("Done")

    def onEnd(self):
        """Called just after last execute OR preemption."""
        return True

class my_skill(SkillBase):
    """
    Tree is:
    ----->:Skill (->)
    ------->:MyPrimitive

    """
    def createDescription(self):
        self.setDescription(MySkill(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Serial())
        skill(
            self.skill("MyPrimitive", "my_primitive")
        )