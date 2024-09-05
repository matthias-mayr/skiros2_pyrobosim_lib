from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, Serial, SerialStar
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class Problem1Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("ObjectStartLocation", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("ObjectTargetLocation", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

#################################################################################
# Implementations
#################################################################################

class problem_1_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem1Solution(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("NavigateExecution", "", remap={"Target": "ObjectStartLocation"}),
            self.skill("PickExecution", ""),
            self.skill("NavigateExecution", "", remap={"Target": "ObjectTargetLocation"}),
            self.skill("PlaceExecution", "")
        )
