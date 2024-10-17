from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, Selector, Serial, RetryOnFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from .pyrobosim_compound_skills import Navigate, Pick, Place, OpenOpenableLocation, CloseOpenableLocation
from .problem_1_fetch_item import Problem1
from .problem_2_waste_and_doors import Problem2
from .problem_3_table_setting import Problem3
from .problem_4_charge import Problem4


#################################################################################
# Implementations
#################################################################################

class problem_1_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem1(), "Problem 1 Solution - Fetch Item")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectStartLocation"}),
            self.skill("OpenLocation", "", remap={"Location": "ObjectStartLocation"}),
            self.skill("Pick", ""),
            self.skill("Navigate", "", remap={"StartLocation": "ObjectStartLocation", "TargetLocation": "ObjectTargetLocation"}),
            self.skill("Place", "", remap={"PlacingLocation": "ObjectTargetLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
        )

class problem_2_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem2(), "Problem 2 Solution - Waste Disposal")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill(SerialStar())(
                self.skill("NavigateAndOpenDoor", "navigate_and_open_doors", remap={"TargetLocation": "Dumpster"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("OpenLocation", "", remap={"OpenableLocation": "Dumpster"}),
            ),
            self.skill(SerialStar())(
                self.skill("Problem1", "", remap={"Object": "Object1", "ObjectTargetLocation": "Dumpster"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            self.skill(SerialStar())(
                self.skill("Problem1", "", remap={"Object": "Object2", "ObjectTargetLocation": "Dumpster"}),
            ),
            self.skill("CloseLocation", "", remap={"OpenableLocation": "Dumpster"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
        )

class problem_3_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem3(), "Problem 3 Solution - Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill(SerialStar())(
                self.skill("Problem1", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
                self.skill("Navigate", "", remap={"TargetLocation": "Pantry"}),
                self.skill("CloseLocation", "", remap={"Location": "Pantry"}),
                # Unset some blackboard parameters to avoid conflicts
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            self.skill(SerialStar())(
                self.skill("Problem1", "", remap={"ObjectTargetLocation": "Table", "Object": "Butter"}),
                self.skill("Navigate", "", remap={"TargetLocation": "Fridge"}),
                self.skill("CloseLocation", "", remap={"Location": "Fridge"}),
            )
        )


class navigate_with_retry_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Navigate(), "Navigate to Location with Retry Solution")

    def expand(self, skill):
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("NavigateExecution", ""),
            ),
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "TargetLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
        )

class pick_with_retry_solution(SkillBase):
    def createDescription(self):
        self.setDescription(Pick(), "Pick Part with Retry Solution")

    def expand(self, skill):
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("PickExecution", ""),
            ),
            self.skill("WmMoveObject", "wm_move_object",
                remap={"StartLocation": "Container", "TargetLocation": "Gripper"}),
        )

class place_with_retry_solution(SkillBase):
    def createDescription(self):
        self.setDescription(Place(), "Place Part with Retry Solution")

    def expand(self, skill):
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("PlaceExecution", ""),
            ),
            self.skill("WmMoveObject", "wm_move_object",
                remap={"StartLocation": "Gripper", "TargetLocation": "PlacingLocation"}),
        )

class open_openablelocation_with_retry_solution(SkillBase):
    def createDescription(self):
        self.setDescription(OpenOpenableLocation(), "Open Location with Retry Solution")

    def expand(self, skill):
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("OpenExecution", ""),
            ),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": True}}),
        )

class close_openablelocation_with_retry_solution(SkillBase):
    def createDescription(self):
        self.setDescription(CloseOpenableLocation(), "Close OpenableLocation with Retry Solution")

    def expand(self, skill):
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("CloseExecution", ""),
            ),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": False}}),
        )


class problem_4_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem4(), "Problem 4 Solution - Waste Disposal and Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Charge", "charge_and_open_doors"),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("Problem3", ""),
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            self.skill("Problem2", ""),
        )

class navigate_with_retry_and_battery_check_solution(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Navigate(), "Navigate to Location")

    def expand(self, skill):
        skill.setProcessor(Serial())
        skill(
            self.skill("BatteryCheckAndCharge"),
            self.skill(SerialStar())(
                self.skill(RetryOnFail(10))(
                    self.skill("NavigateExecution", ""),
                ),
                self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "TargetLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
            )
        )