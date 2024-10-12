from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFf, SerialStar, Selector, Serial, RetryOnFail
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################


class Problem4(SkillDescription):
    def createDescription(self):
        # ======= Params =========
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


class BatteryCheckAndCharge(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("MinBatteryLevel", 40.0, ParamTypes.Required)
        self.addParam("ChargerLocation", Element("skiros:Charger"), ParamTypes.Optional)




#################################################################################
# Implementations
#################################################################################

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
