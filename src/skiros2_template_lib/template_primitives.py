from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase

#################################################################################
# Descriptions
#################################################################################
    
class MyPrimitive(SkillDescription):
    def createDescription(self):
        self._type = ":MyPrimitive"
        #=======Params=========
        self.addParam("WorldModelObject", Element("skiros:TransformationPose"), ParamTypes.World)
        self.addParam("Optional", Element("skiros:TransformationPose"), ParamTypes.Optional)
        self.addParam("Dictionary", {}, ParamTypes.Config)
        self.addParam("Boolean", False, ParamTypes.Config)
        self.addParam("Number", 0.0, ParamTypes.Config)
        

#################################################################################
# Implementations
#################################################################################

class my_primitive(PrimitiveBase):
    """
    This primitive has 3 states
    """
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(MyPrimitive(), self.__class__.__name__)
        
    def onInit(self):
        """Called once when loading the primitive. If return False, the primitive is not loaded"""
        return True

    def onReset(self):
        """Re-initialize the primitive"""
        pass
        
    def onPreempt(self):
        """ Called when skill is requested to stop. """
        pass
    
    def onStart(self):
        """Called just before 1st execute"""
        return self.step("Start")
        
    def execute(self):
        """ Main execution function """
        if self._progress_code<2:
            return self.step("Step")
        else:
            return self.success("Done")
        
    def onEnd(self):
        """Called just after last execute"""
        pass