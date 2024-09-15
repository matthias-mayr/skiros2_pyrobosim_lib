from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, Serial, SerialStar, Selector
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class Problem1Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("ObjectStartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("ObjectTargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

        #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectStartLocation", "Object", True))

class Problem2Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("ObjectStartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("ObjectTargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)

        #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectStartLocation", "Object", True))

#################################################################################
# Implementations
#################################################################################

class problem_1_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem1Solution(), "Problem 1 Solution - Fetch Item")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectStartLocation"}),
            self.skill("Pick", ""),
            self.skill("Navigate", "", remap={"StartLocation": "ObjectStartLocation", "TargetLocation": "ObjectTargetLocation"}),
            self.skill("Place", "", remap={"PlacingLocation": "ObjectTargetLocation"})
        )


class problem_2_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem2Solution(), "Problem 2 Solution - Waste Disposal")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectTargetLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill(Selector())(
                self.skill("OpenLocation", "", remap={"OpenableLocation": "ObjectTargetLocation"}),
                self.skill("Success", ""),
            ),
            self.skill("BbUnsetParam", "", remap={"Parameter": "OpenableLocation"}),
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectStartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("Pick", ""),
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectTargetLocation"}),
            self.skill("Place", "", remap={"PlacingLocation": "ObjectTargetLocation"}),
            self.skill("CloseLocation", "", remap={"OpenableLocation": "ObjectTargetLocation"}),
        )