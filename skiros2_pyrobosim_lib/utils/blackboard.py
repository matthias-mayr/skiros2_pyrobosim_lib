from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


class BbUnsetParam(SkillDescription):
    """
    @brief Removes a parameter from the blackboard, e.g. to trigger the autoparameterization
    """

    def createDescription(self):
        # =======Params=========
        self.addParam("Parameter", Element("sumo:Object"), ParamTypes.Required)


class bb_unset_param(PrimitiveBase):
    def createDescription(self):
        self.setDescription(BbUnsetParam(), self.__class__.__name__)

    def execute(self):
        param = self.params["Parameter"]
        param.unset()
        return self.success("Unset succeeded")