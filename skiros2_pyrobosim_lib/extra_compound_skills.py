from skiros2_skill.core.skill import SkillDescription, SkillBase
from skiros2_skill.core.processors import SerialStar, Selector, Serial, RetryOnFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from .basic_compound_skills import Navigate

#################################################################################
# Descriptions
#################################################################################

class OpenHallwayDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Door", Element("skiros:Door"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Door", "=", False, True))
        # =======PostConditions=========
        self.addPostCondition(self.getPropCond("IsOpen", "skiros:Open", "Door", "=", True, True))
        # Planning book-keeping conditions:
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Door", "=", True, False))

class CloseHallwayDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Door", Element("skiros:Door"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Door", "=", True, True))
        # =======PostConditions=========
        self.addPostCondition(self.getPropCond("IsOpen", "skiros:Open", "Door", "=", False, True))
        # Planning book-keeping conditions:
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Door", "=", False, False))

class Charge(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "ChargerLocation", True))
        # Planning book-keeping conditions:
        self.addPostCondition(self.getRelationCond("NoRobotAt", "skiros:at", "Robot", "StartLocation", False))

class OpenLocation(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "Location", True))

class CloseLocation(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Inferred)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "Location", True))

class BatteryCheckAndCharge(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("MinBatteryLevel", 40.0, ParamTypes.Required)
        self.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Optional)


class NavigateAndOpenDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "TargetLocation", True))


#################################################################################
# Implementations
#################################################################################

class open_hallway_door(SkillBase):
    def createDescription(self):
        self.setDescription(OpenHallwayDoor(), "Open Hallway Door")

    def expand(self, skill):
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "Door"}),
            self.skill("OpenLocation", "", remap={"Location": "Door"}),
        )

class close_hallway_door(SkillBase):
    def createDescription(self):
        self.setDescription(CloseHallwayDoor(), "Close Hallway Door")

    def expand(self, skill):
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "Door"}),
            self.skill("CloseLocation", "", remap={"Location": "Door"}),
        )

class charge(SkillBase):
    def createDescription(self):
        self.setDescription(Charge(), "Navigate to Charger")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("NavigateExecution", "", remap={"TargetLocation": "ChargerLocation"}),
            ),
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "ChargerLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
        )

class open_location(SkillBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), "Open Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Location", "=", False, True))

    def expand(self, skill):
        skill(
            self.skill("OpenOpenableLocation", "", remap={"OpenableLocation": "Location"}),
        )

class skip_open_openablelocation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), "Skip to Open OpenableLocation")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsOpen", "skiros:Open", "Location", "=", True, True))

    def execute(self):
        return self.success("Skipped opening openablelocation that is already open")

class skip_open_location(PrimitiveBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), "Skip to Open Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", False))

    def execute(self):
        return self.success("Skipped opening location that can not be opened")
    
class close_location(SkillBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), "Close Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Location", "=", True, True))

    def expand(self, skill):
        skill(
            self.skill("CloseOpenableLocation", "", remap={"OpenableLocation": "Location"}),
        )

class skip_close_openablelocation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), "Skip to Close OpenableLocation")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsOpen", "skiros:Open", "Location", "=", False, True))

    def execute(self):
        return self.success("Skipped closing openablelocation that is already closed")

class skip_close_location(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), "Skip to Close Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", False))

    def execute(self):
        return self.success("Skipped closing location that can not be closed")


class navigate_and_open_door(SkillBase):
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(NavigateAndOpenDoor(), "Navigate and Open Single Door")
    
    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Navigate", ""),
            self.skill(Selector())(
                self.skill("LocationIsDoor", "", remap={"Location": "TargetLocation"}, specify={"ReverseResult": True, "Open": False}),
                self.skill("OpenOpenableLocation", "", remap={"OpenableLocation": "TargetLocation"}),
            ),
        )

class navigate_and_open_doors(SkillBase):
    def createDescription(self):
        self.setDescription(NavigateAndOpenDoor(), "Navigate and Open Doors")

    def modifyDescription(self, skill):
        skill.addParam("IntermediateLocation", Element("skiros:Location"), ParamTypes.Optional)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("BbUnsetParam", "", remap={"Parameter": "IntermediateLocation"}),
            self.skill(Serial())(
                self.skill(Selector())(
                    self.skill("IsNone", "", remap={"Param": "IntermediateLocation"}),
                    self.skill(SerialStar())(
                        self.skill("NavigateAndOpenDoor", "navigate_and_open_door", remap={"TargetLocation": "IntermediateLocation"}),
                        self.skill("CopyValue", "", remap={"Output": "StartLocation", "Input": "IntermediateLocation"}),
                    ),
                ),
                self.skill("SelectDoorsToTarget", "", remap={"Location": "StartLocation"}),
            ),
        )

class battery_check_and_charge(SkillBase):
    def createDescription(self):
        self.setDescription(BatteryCheckAndCharge(), "Battery Check and Charge")

    def expand(self, skill):
        skill.setProcessor(Selector())
        skill(
            self.skill("BatteryAboveLevel", ""),
            self.skill(Serial())(
                self.skill("ChargerLocationFromWM", ""),
                self.skill("Charge", ""),
            )
        )
