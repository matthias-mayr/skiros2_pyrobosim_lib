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


