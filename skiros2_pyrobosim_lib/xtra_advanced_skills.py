from skiros2_skill.core.skill import SkillDescription, SkillBase, ParallelFs, ParallelFf, SerialStar
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class MoveAllObjects(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("InitialLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)


class SelectObjectToFetch(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Optional)
        # =======PreConditions=========

class OpenLocation(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "Location", True))

class CloseLocation(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Inferred)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "Location", True))


#################################################################################
# Implementations
#################################################################################

class move_all_objects(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(MoveAllObjects(), "Move All Objects from Initial to Target Location")

    def expand(self, skill):
        skill.setProcessor(ParallelFs())
        skill(
            self.skill("SelectObjectToFetch", "", remap={"Location": "InitialLocation", "TargetLocation": "Table"}),
            self.skill(ParallelFf())(
                self.skill("SelectObjectToFetch", "", remap={"Location": "InitialLocation", "TargetLocation": "Table"}),
                self.skill(SerialStar())(
                    self.skill("Problem1Solution", "", remap={"ObjectStartLocation": "InitialLocation", "ObjectTargetLocation": "Table"}),
                    self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                )
            ),
        )


class select_object_to_fetch(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SelectObjectToFetch(), "Select Object To Fetch")

    def onPreempt(self):
        """ Called when skill is requested to stop. """
        return self.success("Stopped")
    
    def onStart(self):
        self.current_object = None
        return True
    
    def get_elements_contained_by_location(self, location):
        location_element = self._wmi.get_element(location.id)
        contain_relations = location_element.getRelations(pred="skiros:contain", subj='-1')
        contained = [v["dst"] for v in contain_relations]
        return contained

    def execute(self):
        """ Main execution function. Should return with either: self.fail, self.step or self.success """
        if self.current_object:
            # Check if the object is at the target location
            contained_objects = self.get_elements_contained_by_location(self.params["TargetLocation"].value)
            if self.current_object.id not in contained_objects:
                return self.step(f"Object '{self.current_object.label}' is not at the target location yet")
    
        # We fetch the latest from the WM to make sure that all relations are updated
        contained_objects = self.get_elements_contained_by_location(self.params["Location"].value)
        if len(contained_objects) == 0:
            return self.success("No objects to fetch. We are done!")
        
        object_element = self._wmi.get_element(contained_objects[0])
        self.current_object = object_element
        self.params["Object"].value = object_element
        return self.step(f"Object '{object_element.label}' selected")

class open_location(SkillBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), "Open Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Location", "=", False, True))

    def expand(self, skill):
        skill(
            self.skill("OpenOpenableLocation", "", remap={"OpenableLocation": "Location"}),
        )

class skip_open_openablelocation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), "Skip to Open OpenableLocation")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsOpen", "skiros:Open", "Location", "=", True, True))

    def execute(self):
        return self.success("Skipped opening openablelocation that is already open")

class skip_open_location(PrimitiveBase):
    def createDescription(self):
        self.setDescription(OpenLocation(), "Skip to Open Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", False))

    def execute(self):
        return self.success("Skipped opening location that can not be opened")
    
class close_location(SkillBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), "Close Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsClosed", "skiros:Open", "Location", "=", True, True))

    def expand(self, skill):
        skill(
            self.skill("CloseOpenableLocation", "", remap={"OpenableLocation": "Location"}),
        )

class skip_close_openablelocation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), "Skip to Close OpenableLocation")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", True))
        skill.addPreCondition(self.getPropCond("IsOpen", "skiros:Open", "Location", "=", False, True))

    def execute(self):
        return self.success("Skipped closing openablelocation that is already closed")

class skip_close_location(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CloseLocation(), "Skip to Close Location")

    def modifyDescription(self, skill):
        skill.addPreCondition(self.getHasPropCond("HasOpen", "skiros:Open", "Location", False))

    def execute(self):
        return self.success("Skipped closing location that can not be closed")
