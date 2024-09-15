from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, ParallelFf, SerialStar, Selector
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


class Problem3Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Table", Element("skiros:Table"), ParamTypes.Required)
        self.addParam("Fridge", Element("skiros:Fridge"), ParamTypes.Required)
        self.addParam("Pantry", Element("skiros:Pantry"), ParamTypes.Required)

        # Todo: Add post conditions:
        # - Fridge & pantry must be closed

class MoveAllObjects(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("InitialLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)

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
            self.skill("OpenLocation", "", remap={"Location": "ObjectStartLocation"}),
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

class problem_3_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem3Solution(), "Problem 3 Solution - Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill(SerialStar())(
                self.skill("MoveAllObjects", "", remap={"InitialLocation": "Fridge", "TargetLocation": "Table"}),
                self.skill("Navigate", "", remap={"TargetLocation": "Fridge"}),
                self.skill("CloseLocation", "", remap={"Location": "Fridge"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "InitialLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            self.skill(SerialStar())(
                self.skill("MoveAllObjects", "", remap={"InitialLocation": "Pantry", "TargetLocation": "Table"}),
                self.skill("Navigate", "", remap={"TargetLocation": "Pantry"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("CloseLocation", "", remap={"Location": "Pantry"}),
            )
        )


class move_all_objects(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(MoveAllObjects(), "Move All Objects from Initial to Target Location")

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("SelectObjectToFetch", "", remap={"Location": "InitialLocation", "TargetLocation": "Table"}),
            self.skill(ParallelFf())(
                self.skill("SelectObjectToFetch", "", remap={"Location": "InitialLocation", "TargetLocation": "Table"}),
                self.skill(SerialStar())(
                    self.skill("Problem1Solution", "", remap={"ObjectStartLocation": "InitialLocation", "ObjectTargetLocation": "Table"}),
                    self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                )
            ),
        )