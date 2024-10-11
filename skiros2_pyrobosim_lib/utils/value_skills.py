from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class IsNone(SkillDescription):
    def createDescription(self):
        self.addParam("Param", Element("sumo:Object"), ParamTypes.Optional)

class CopyValue(SkillDescription):
    def createDescription(self):
        self.addParam("Input", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Output", Element("skiros:Location"), ParamTypes.Optional)


#################################################################################
# Implementations
#################################################################################

class is_none(PrimitiveBase):
    def createDescription(self):
        self.setDescription(IsNone(), "Is None")

    def execute(self):
        if self.params["Param"].value is None or self.params["Param"].value.id == '':
            return self.success("Param was none")
        return self.fail(f"Param is {self.params["Param"].value}", -1)

class copy_value(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CopyValue(), "Copy Value")

    def execute(self):
        self.params["Output"].value = self.params["Input"].value
        return self.success(f"Set parameter to {self.params["Output"].value.label}")

