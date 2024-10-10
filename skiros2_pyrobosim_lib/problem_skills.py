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

        # =======PreConditions=========
        # Precondition stating that 'ObjectStartLocation' must contain the object. Since we know the 'Object' from the required parameter, we can automatically fill out the 'ObjectStartLocation' with this rule:
        self.addPreCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectStartLocation", "Object", True))
        # =======PostConditions=========
        # Finally a postcondition stating that after the Problem1 skill ran, the object must be contained by the 'ObjectTargetLocation'
        self.addPostCondition(self.getRelationCond("ObjectContained", "skiros:contain", "ObjectTargetLocation", "Object", True))

class Problem2(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Dumpster", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Waste1", Element("skiros:Waste"), ParamTypes.Required)
        self.addParam("Waste2", Element("skiros:Waste"), ParamTypes.Required)

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

class Problem4(SkillDescription):
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

#################################################################################
# Implementations
#################################################################################

class problem_1(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(description=Problem1(), label="Problem 1 - Fetch Item")

    def expand(self, skill):
        # We want to execute some skills in a sequence and remember which one we executed, so we use SerialStar. It will abort the skill if any of the child skills fail:
        skill.setProcessor(SerialStar())
        skill(
            # As a first step, we want to navigate to the location where the object is currently located. In our skill, this is saved in the 'ObjectStartLocation' parameter.
            # However, the 'Navigate' skill takes only a parameter 'TargetLocation', so we need to remap/rewire those parameters like this:
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectStartLocation"}),
            # FIXME 1: Add more skills to solve this task
            # From here, feel free to add more skills. Check out 'basic_compound_skills.py' to see which ones are available.




            # This is for temporary cleanup - just ignore its existence:
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
        )



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
