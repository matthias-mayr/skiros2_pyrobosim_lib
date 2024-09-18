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
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Object", Element("skiros:Part"), ParamTypes.Optional)
        # =======PreConditions=========


class SelectDoorsToTarget(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Door", Element("skiros:OpenableLocation"), ParamTypes.Optional)


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


class select_doors_to_target(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SelectDoorsToTarget(), "Select Closest Door Leading to Target")

    def onPreempt(self):
        return self.success("Stopped")

    def find_path_between(self, start, target):
        goal_found = False
        visited_from = dict()
        queue = [(None, None, start)]

        while queue:
            from_elm, door, elm = queue.pop(0)

            if elm.id in visited_from:
                continue

            visited_from[elm.id] = (from_elm, door)

            if elm.id == target.id:
                goal_found = True
                break

            for door, to_elm in self.doors_from(elm):
                queue.append((elm, door, to_elm))

        path = []
        if not goal_found:
            return path

        elm = visited_from[target.id]
        while elm is not None:
            path.append(elm)
            elm = visited_from[elm.id]

        return list(reversed(path))

    def doors_from(self, elm):
        for relation in elm.getRelations(obj='-1', pred='skiros:adjacentLocation'):
            door = self.wmi.get_element(relation['src'])
            for door_relation in door.getRelations(subj='-1', pred='skiros:adjacentLocation'):
                yield door, self.wmi.get_element(door_relation['dst'])

    def onStart(self):
        self.current_door = None
        self.path = self.find_path_between(
            self.params["Location"],
            self.params["TargetLocation"],
        )
        return True

    def execute(self):
        if self.current_door and self.current_door.id != self.params["Location"].id:
            return self.step(f"Current location is not {self.current_door.label}")

        if not self.path:
            return self.success("Path has been completed")

        self.current_door = self.path[0]
        self.path = self.path[1:]
        self.params["Location"].value = self.current_door
        return self.step(f"Door '{self.current_door.label}' selected")

