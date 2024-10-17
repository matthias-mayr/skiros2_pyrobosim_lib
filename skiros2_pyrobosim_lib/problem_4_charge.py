from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFf, SerialStar, Selector, Serial, RetryOnFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from .pyrobosim_compound_skills import Navigate

#################################################################################
# Descriptions
#################################################################################


class Problem4(SkillDescription):
    def createDescription(self):
        # ======= Params =========
        self.addParam("Dumpster", Element("skiros:Dumpster"), ParamTypes.Required)
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


class Charge(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        self.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Required)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "ChargerLocation", True))


class BatteryCheckAndCharge(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("MinBatteryLevel", 40.0, ParamTypes.Required)
        self.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Optional)


#################################################################################
# Implementations
#################################################################################

class battery_check_and_charge(SkillBase):
    def createDescription(self):
        self.setDescription(BatteryCheckAndCharge(), "Battery Check and Charge")

    def expand(self, skill):
        skill.setProcessor(Selector())
        skill(
            # We check whether the battery is above the minimum level
            self.skill("BatteryAboveLevel", ""),
            # If the battery is below the minimum level, we charge the robot:
            self.skill(Serial())(
                # This sets the charger location to the charger location in the blackboard:
                self.skill("ChargerLocationFromWM", ""),
                # This navigates to the charger location:
                self.skill("Charge", "charge_directly"),
            )
        )


class navigate_with_retry_and_battery_check(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Navigate(), "Navigate to Location")

    def expand(self, skill):
        # Previously we were executing everything with a SerialStar processor. If we would use that for the battery check, we would only check it once at the beginning of the skill. However, we want to check the battery level constantly, so we set it to a Serial processor:
        skill.setProcessor(Serial())
        skill(
            # FIXME 4.1: Add the battery check and charge skill here:
            
            # Now we can plug in our previous navigation skill:
            self.skill(SerialStar())(
                self.skill(RetryOnFail(10))(
                    self.skill("NavigateExecution", ""),
                ),
                self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "TargetLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
            )
        )

class problem_4(SkillBase):
    """
    """
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(Problem4(), "Problem 4 - Waste Disposal and Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            # FIXME 4.2: Charge the robot before starting the task and make sure that the path to the charger is not blocked by doors.
            # You can look at the new skills that were introduced in this file and think about which one should be included here.

            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            # FIXME 4.3: Before using butter it's often a good idea to have it warm up a bit, so we will set the table first. Try to reuse what you did in Problem3 to set the table:



            # After setting the table, we need to remove some parameters from the blackboard. Nothing to do here:
            self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
            self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            # FIXME 4.4: Now we can fetch the waste and dispose of it. Try to re-use a skill from the previous problems to do this:

        )

class charge_directly(SkillBase):
    def createDescription(self):
        self.setDescription(Charge(), "Navigate Directly to Charger")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill(RetryOnFail(10))(
                self.skill("NavigateExecution", "", remap={"TargetLocation": "ChargerLocation"}),
            ),
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "ChargerLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
        )

class charge_and_open_doors(SkillBase):
    def createDescription(self):
        self.setDescription(Charge(), "Navigate to Charger and Open Doors")

    def modifyDescription(self, skill):
        skill.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Optional)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("ChargerLocationFromWM", ""),
            self.skill("NavigateAndOpenDoor", "navigate_and_open_doors", remap={"TargetLocation": "ChargerLocation"}),
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "ChargerLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
        )


