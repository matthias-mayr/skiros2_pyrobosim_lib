from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class SelectObjectToFetch(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Optional)
        # =======PreConditions=========


#################################################################################
# Implementations
#################################################################################

class select_object_to_fetch(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SelectObjectToFetch(), "Select Object To Fetch")

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.success("Stopped")
    
    def onStart(self):
        return True
    
    def get_elements_contained_by_location(self, location):
        location_element = self._wmi.get_element(location.id)
        contain_relations = location_element.getRelations(pred="skiros:contain", subj='-1')
        contained = [v["dst"] for v in contain_relations]
        return contained

    def execute(self):
        """ Main execution function. Should return with either: self.fail, self.step or self.success """
        # We fetch the latest from the WM to make sure that all relations are updated
        contained_objects = self.get_elements_contained_by_location(self.params["Location"].value)
        if len(contained_objects) == 0:
            return self.success("No objects to fetch. We are done!")
        
        object_element = self._wmi.get_element(contained_objects[0])
        self.params["Object"].value = object_element
        return self.step(f"Object {object_element.label} selected")