from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, ParallelFf, SerialStar, RetryOnFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from .pyrobosim_compound_skills import Navigate, Pick, Place, OpenOpenableLocation, CloseOpenableLocation

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
            # FIXME 3.2: We will fetch the bread here and later the butter
            self.skill(SerialStar())(
                # As an example, we reuse our Problem1 skill to fetch an item. The Problem1 skill has parameter 'Object' and 'ObjectTargetLocation' that we need to set. If you check the Problem3 skill description you will see that we have parameters with names like "Bread" and "Table" that we need to remap like this:
                self.skill("Problem1", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
                # FIXME 3.2: Now add skills to close the pantry


                # Unset some blackboard parameters to avoid conflicts. Nothing to do here.
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            # FIXME 3.3: Now fetch the butter and close the fridge
            self.skill(SerialStar())(
                # FIXME 3.3: Add more skills to solve this task. Try to use a previous skill
            )
        )


class navigate_with_retry(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Navigate(), "Navigate to Location with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3.1: Make this 'NavigateExecution' skill retryable
            self.skill("NavigateExecution", ""),
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "TargetLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
        )

class pick_with_retry(SkillBase):
    def createDescription(self):
        self.setDescription(Pick(), "Pick Part with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3.1: Make this next skill execution retryable
            self.skill("PickExecution", ""),
            self.skill("WmMoveObject", "wm_move_object",
                remap={"StartLocation": "Container", "TargetLocation": "Gripper"}),
        )

class place_with_retry(SkillBase):
    def createDescription(self):
        self.setDescription(Place(), "Place Part with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3.1: Make this next skill execution retryable
            self.skill("PlaceExecution", ""),
            self.skill("WmMoveObject", "wm_move_object",
                remap={"StartLocation": "Gripper", "TargetLocation": "PlacingLocation"}),
        )

class open_openablelocation_with_retry(SkillBase):
    def createDescription(self):
        self.setDescription(OpenOpenableLocation(), "Open Location with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3.1: Make this next skill execution retryable
            self.skill("OpenExecution", ""),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": True}}),
        )

class close_openablelocation_with_retry(SkillBase):
    def createDescription(self):
        self.setDescription(CloseOpenableLocation(), "Close OpenableLocation with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3.1: Make this next skill execution retryable
            self.skill("CloseExecution", ""),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": False}}),
        )
