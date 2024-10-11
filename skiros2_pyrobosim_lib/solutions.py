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

        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectStartLocation", "Object", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectTargetLocation", "Object", True))

class Problem2Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Dumpster", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Waste1", Element("skiros:Waste"), ParamTypes.Required)
        self.addParam("Waste2", Element("skiros:Waste"), ParamTypes.Required)

class Problem3Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Table", Element("skiros:Table"), ParamTypes.Required)
        self.addParam("Bread", Element("skiros:Bread"), ParamTypes.Required)
        self.addParam("Butter", Element("skiros:Butter"), ParamTypes.Required)
        self.addParam("Fridge", Element("skiros:Fridge"), ParamTypes.Inferred)
        self.addParam("Pantry", Element("skiros:Pantry"), ParamTypes.Inferred)

        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("FridgeContainsButter", "skiros:contain", "Fridge", "Butter", True))
        self.addPreCondition(self.getRelationCond("PantryContainsBread", "skiros:contain", "Pantry", "Bread", True))

        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("TableContainsBread", "skiros:contain", "Table", "Bread", True))
        self.addPostCondition(self.getRelationCond("TableContainsButter", "skiros:contain", "Table", "Butter", True))
        self.addPostCondition(self.getPropCond("FridgeClosed", "skiros:Open", "Fridge", "=", False, True))
        self.addPostCondition(self.getPropCond("PantryClosed", "skiros:Open", "Pantry", "=", False, True))

class Problem4Solution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Dumpster", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Waste1", Element("skiros:Waste"), ParamTypes.Required)
        self.addParam("Waste2", Element("skiros:Waste"), ParamTypes.Required)
        self.addParam("Table", Element("skiros:Table"), ParamTypes.Required)
        self.addParam("Bread", Element("skiros:Bread"), ParamTypes.Required)
        self.addParam("Butter", Element("skiros:Butter"), ParamTypes.Required)
        self.addParam("Fridge", Element("skiros:Fridge"), ParamTypes.Inferred)
        self.addParam("Pantry", Element("skiros:Pantry"), ParamTypes.Inferred)

        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("FridgeContainsButter", "skiros:contain", "Fridge", "Butter", True))
        self.addPreCondition(self.getRelationCond("PantryContainsBread", "skiros:contain", "Pantry", "Bread", True))

        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("TableContainsBread", "skiros:contain", "Table", "Bread", True))
        self.addPostCondition(self.getRelationCond("TableContainsButter", "skiros:contain", "Table", "Butter", True))
        self.addPostCondition(self.getPropCond("FridgeClosed", "skiros:Open", "Fridge", "=", False, True))
        self.addPostCondition(self.getPropCond("PantryClosed", "skiros:Open", "Pantry", "=", False, True))

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
            self.skill("Place", "", remap={"PlacingLocation": "ObjectTargetLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
        )


class problem_2_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem2Solution(), "Problem 2 Solution - Waste Disposal")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill(SerialStar())(
                self.skill("Navigate", "navigate_and_open_doors", remap={"TargetLocation": "Dumpster"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill(Selector())(
                    self.skill("OpenLocation", "", remap={"OpenableLocation": "Dumpster"}),
                    self.skill("Success", ""),
                ),
            ),
            self.skill(SerialStar())(
                self.skill("Problem1Solution", "", remap={"Object": "Object1", "ObjectTargetLocation": "Dumpster"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            self.skill(SerialStar())(
                self.skill("Problem1Solution", "", remap={"Object": "Object2", "ObjectTargetLocation": "Dumpster"}),
            ),
            self.skill("CloseLocation", "", remap={"OpenableLocation": "Dumpster"}),
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
                self.skill("Problem1Solution", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
                self.skill("Navigate", "", remap={"TargetLocation": "Pantry"}),
                self.skill("CloseLocation", "", remap={"Location": "Pantry"}),
                # Unset some blackboard parameters to avoid conflicts
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            self.skill(SerialStar())(
                self.skill("Problem1Solution", "", remap={"ObjectTargetLocation": "Table", "Object": "Butter"}),
                self.skill("Navigate", "", remap={"TargetLocation": "Fridge"}),
                self.skill("CloseLocation", "", remap={"Location": "Fridge"}),
            )
        )

class problem_4_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem4Solution(), "Problem 4 Solution - Waste Disposal and Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Problem3Solution", ""),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
            self.skill("Problem2Solution", ""),
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