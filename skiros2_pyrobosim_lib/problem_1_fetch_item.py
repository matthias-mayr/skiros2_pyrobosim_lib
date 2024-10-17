from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, ParallelFf, SerialStar, Selector
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class Problem1(SkillDescription):
    def createDescription(self):
        # =======Params=========
        # A required parameter 'Object' that is a world-model element of the type 'Part':
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Required)
        # Required parameter on where to bring the object of type world-model element 'Location':
        self.addParam("ObjectTargetLocation", Element("skiros:Location"), ParamTypes.Required)
        # An inferred parameter on the start (current) location of the object since we go there first and need to know where to go:
        self.addParam("ObjectStartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("Gripper", Element("rparts:GripperEffector"), ParamTypes.Inferred)

        # =======PreConditions=========
        # Precondition stating that 'ObjectStartLocation' must contain the object. Since we know the 'Object' from the required parameter, we can automatically fill out the 'ObjectStartLocation' with this rule:
        self.addPreCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectStartLocation", "Object", True))
        self.addPreCondition(self.getRelationCond("RobotHasAGripper", "skiros:hasA", "Robot", "Gripper", True))
        self.addPreCondition(self.getPropCond("EmptyHanded", "skiros:ContainerState", "Gripper", "=", "Empty", True))
        self.addPreCondition(self.getPropCond("IsReachable", "skiros:Reachable", "ObjectTargetLocation", "=", True, True))
        self.addPreCondition(self.getPropCond("ObjectTargetLocationIsOpen", "skiros:Open", "ObjectTargetLocation", "=", True, True))
        # =======PostConditions=========
        # Finally a postcondition stating that after the Problem1 skill ran, the object must be contained by the 'ObjectTargetLocation'
        self.addPostCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectTargetLocation", "Object", True))
        self.addPostCondition(self.getRelationCond("RobotAtTargetLocation", "skiros:at", "Robot", "ObjectTargetLocation", True))
        self.addPostCondition(self.getPropCond("ObjectStartLocationIsOpen", "skiros:Open", "ObjectStartLocation", "=", True, True))


#################################################################################
# Implementations
#################################################################################

class problem_1(SkillBase):
    """
    """
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(description=Problem1(), label="Problem 1 - Fetch Item")

    def expand(self, skill):
        # We want to execute some skills in a sequence and remember which one we executed, so we use SerialStar. It will abort the skill if any of the child skills fail:
        skill.setProcessor(SerialStar())
        skill(
            # As a first step, we want to navigate to the location where the object is currently located. In our skill, this is saved in the 'ObjectStartLocation' parameter.
            # However, the 'Navigate' skill takes only a parameter 'TargetLocation', so we need to remap/rewire those parameters like this:
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectStartLocation"}),
            # FIXME 1: Add more skills to solve this task
            # From here, feel free to add more skills. Check out 'pyrobosim_compound_skills.py' to see which ones are available.




            # This is for temporary cleanup - just ignore its existence:
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
        )
