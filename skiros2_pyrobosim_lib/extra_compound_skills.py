from skiros2_skill.core.skill import SkillDescription, SkillBase
from skiros2_skill.core.processors import RetryOnFail, SerialStar, ParallelFs
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element

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

        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "Location", True))


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
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "ChargerLocation"}),
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
