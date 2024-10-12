from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, ParallelFf, SerialStar, Selector
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class Problem3(SkillDescription):
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


#################################################################################
# Implementations
#################################################################################

class problem_3(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem3(), "Problem 3 - Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            # FIXME 3: We will fetch the bread here and later the butter
            self.skill(SerialStar())(
                # FIXME 3: Add more skills to solve this task. Try to use a previous skill
                self.skill("Problem1Solution", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
                

                # FIXME 3: Close the pantry


                # Unset some blackboard parameters to avoid conflicts
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            # FIXME 3: Now fetch the butter
            self.skill(SerialStar())(
                # FIXME 3: Add more skills to solve this task. Try to use a previous skill
            )
        )
