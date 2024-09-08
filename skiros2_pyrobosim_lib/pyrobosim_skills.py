from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, Serial, SerialStar
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class Navigate(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "TargetLocation", True))
        # Planning book-keeping conditions:
        self.addPostCondition(self.getRelationCond("NoRobotAt", "skiros:at", "Robot", "StartLocation", False))

class Pick(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Container", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Inferred)
        #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotHasAGripper", "skiros:hasA", "Robot", "Gripper", True))
        self.addPreCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", True))
        self.addPreCondition(self.getRelationCond("ObjectInContainer", "skiros:contain", "Container", "Object", True))
        #=======HoldConditions=========
        self.addHoldCondition(self.getRelationCond("RobotAtLocation", "skiros:at", "Robot", "Container", True))
        #=======PostConditions=========
        self.addPostCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", False))
        self.addPostCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True))

class Place(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("PlacingLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Inferred)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Inferred)
        #=======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotHasAGripper", "skiros:hasA", "Robot", "Gripper", True))
        self.addPreCondition(self.getRelationCond("Holding", "skiros:contain", "Gripper", "Object", True))
        #=======HoldConditions=========
        self.addHoldCondition(self.getRelationCond("RobotAtLocation", "skiros:at", "Robot", "PlacingLocation", True))
        #=======PostConditions=========
        self.addPostCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", True))
        self.addPostCondition(self.getRelationCond("NotHolding", "skiros:contain", "Gripper", "Object", False))
        self.addPostCondition(self.getRelationCond("InPlace", "skiros:contain", "PlacingLocation", "Object", True))

class OpenLocation(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("OpenableLocation", Element("skiros:OpenableLocation"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "OpenableLocation", True))
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", False, True))
        # =======PostConditions=========
        self.addPostCondition(self.getPropCond("IsOpen", "skiros:Open", "OpenableLocation", "=", True, True))
        # Planning book-keeping conditions:
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", True, False))

class OpenDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("OpenableLocation", Element("skiros:Door"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", False, True))
        # =======PostConditions=========
        self.addPostCondition(self.getPropCond("IsOpen", "skiros:Open", "OpenableLocation", "=", True, True))
        # Planning book-keeping conditions:
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", True, False))

class CloseLocation(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("OpenableLocation", Element("skiros:OpenableLocation"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "OpenableLocation", True))
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", True, True))
        # =======PostConditions=========
        self.addPostCondition(self.getPropCond("IsOpen", "skiros:Open", "OpenableLocation", "=", False, True))
        # Planning book-keeping conditions:
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", False, False))

class CloseDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("OpenableLocation", Element("skiros:Door"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", True, True))
        # =======PostConditions=========
        self.addPostCondition(self.getPropCond("IsOpen", "skiros:Open", "OpenableLocation", "=", False, True))
        # Planning book-keeping conditions:
        self.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "OpenableLocation", "=", False, False))

class Charge(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "TargetLocation", True))
        # Planning book-keeping conditions:
        self.addPostCondition(self.getRelationCond("NoRobotAt", "skiros:at", "Robot", "StartLocation", False))

#################################################################################
# Implementations
#################################################################################

class navigate(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Navigate(), self.__class__.__name__)

    def set_at(self, src, dst, state):
      return self.skill("WmSetRelation", "wm_set_relation",
          remap={'Dst': dst},
          specify={'Src': self.params[src].value, 'Relation': 'skiros:at', 'RelationState': state})

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("NavigateExecution", "", remap={"Target": "ObjectTargetLocation"}),
            self.set_at("Robot", "StartLocation", False),
            self.set_at("Robot", "TargetLocation", True),
        )

class pick(SkillBase):
    def createDescription(self):
        self.setDescription(Pick(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("PickExecution", ""),
            self.skill("WmMoveObject", "wm_move_object",
                remap={"StartLocation": "Container", "TargetLocation": "Gripper"}),
        )

class place(SkillBase):
    def createDescription(self):
        self.setDescription(Place(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("PlaceExecution", ""),
            self.skill("WmMoveObject", "wm_move_object",
                remap={"StartLocation": "Gripper", "TargetLocation": "PlacingLocation"}),
        )

class open_location(SkillBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("OpenExecution", ""),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": True}}),
        )

class open_door(SkillBase):
    def createDescription(self):
        self.setDescription(OpenDoor(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "OpenableLocation"}),
            self.skill("OpenLocation", ""),
        )

class close_location(SkillBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("CloseExecution", ""),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": False}}),
        )

class close_door(SkillBase):
    def createDescription(self):
        self.setDescription(CloseDoor(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "OpenableLocation"}),
            self.skill("CloseLocation", ""),
        )

class charge(SkillBase):
    def createDescription(self):
        self.setDescription(Charge(), self.__class__.__name__)

    def expand(self, skill):
        skill(
            self.skill("Navigate", "", remap={"TargetLocation": "ChargerLocation"}),
        )