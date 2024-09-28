from enum import Enum, auto
import skiros2_common.tools.logger as log
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
        self.addParam("IntermediateLocation", Element("skiros:Location"), ParamTypes.Optional)

class LocationIsDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Open", bool, ParamTypes.Required)
        self.addParam("ReverseResult", False, ParamTypes.Required)
        # =======PostConditions=========
        # is of type door or not

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
    class Edge(Enum):
        door = auto()
        room = auto()
        sublocation = auto()

    def createDescription(self):
        self.setDescription(SelectDoorsToTarget(), "Select Closest Door Leading to Target")

    def onPreempt(self):
        return self.success("Stopped")

    def find_path_between(self, start, target):
        goal_found = False

        log.warn(f"Starting path search from {start}")
        start_type = self.location_type(start)
        if start_type is None:
            log.warn(f"Start location is not a known type {start.label}: {start.type}")
            return []

        visited_from = dict()
        queue = [(0, None, start_type, start)]

        while queue:
            res = min(queue, key=lambda x: x[0])
            queue.remove(res)
            distance, from_elm, elm_type, elm = res

            if elm.id in visited_from:
                prev, _ = visited_from[elm.id]
                if prev < distance:
                    continue

            visited_from[elm.id] = (distance, from_elm)

            if elm.id == target.id:
                goal_found = True
                break

            for to_distance, to_type, to_elm in self.connections_from(elm_type, elm):
                queue.append((distance + to_distance, elm, to_type, to_elm))

        path = [target]
        if not goal_found:
            return path

        _, elm = visited_from[target.id]
        while elm is not None:
            path.append(elm)
            _, elm = visited_from[elm.id]

        log.warn('FOUND PATH: ' + str(list(reversed(path))))
        return list(reversed(path))

    def connections_from(self, elm_type, elm):
        if elm_type is self.Edge.door:
            yield from self.connections_from_door(elm)
        elif elm_type is self.Edge.room:
            yield from self.connections_from_room(elm)
        elif elm_type is self.Edge.sublocation:
            yield from self.connections_from_sublocation(elm)
        else:
            raise NotImplementedError(elm_type)

    def connections_from_door(self, elm):
        for door_relation in elm.getRelations(subj='-1', pred='skiros:adjacentLocation'):
            room = self.wmi.get_element(door_relation['dst'])
            room_type = self.location_type(room)
            if room_type is None:
                continue
            yield 2, room_type, room

            for room_relation in room.getRelations(subj='-1', pred='skiros:contain'):
                sublocation = self.wmi.get_element(room_relation['dst'])
                sublocation_type = self.location_type(sublocation)
                if sublocation_type is None:
                    continue
                yield 2, sublocation_type, sublocation

            for room_relation in room.getRelations(obj='-1', pred='skiros:adjacentLocation'):
                door = self.wmi.get_element(room_relation['src'])
                door_type = self.location_type(door)
                if door_type is None:
                    continue
                yield 1, door_type, door

    def connections_from_room(self, elm):
        for room_relation in elm.getRelations(obj='-1', pred='skiros:adjacentLocation'):
            door = self.wmi.get_element(room_relation['src'])
            door_type = self.location_type(door)
            if door_type is None:
                continue
            yield 1, door_type, door

        for room_relation in elm.getRelations(subj='-1', pred='skiros:contain'):
            sublocation = self.wmi.get_element(room_relation['dst'])
            sublocation_type = self.location_type(sublocation)
            if sublocation_type is None:
                continue
            yield 2, sublocation_type, sublocation

    def connections_from_sublocation(self, elm):
        for sublocation_relation in elm.getRelations(obj='-1', pred='skiros:contain'):
            room = self.wmi.get_element(sublocation_relation['src'])
            room_type = self.location_type(room)
            if room_type is None:
                continue
            yield 2, room_type, room

            for room_relation in room.getRelations(obj='-1', pred='skiros:adjacentLocation'):
                door = self.wmi.get_element(room_relation['src'])
                door_type = self.location_type(door)
                if door_type is None:
                    continue
                yield 1, door_type, door

    def location_type(self, location):
        cls = location.type

        if cls == 'skiros:Door':
            return self.Edge.door
        elif cls == 'skiros:Room':
            return self.Edge.room
        elif cls == 'skiros:Location' or self.wmi.get_super_class(cls) in ['skiros:Location', 'skiros:OpenableLocation']:
            return self.Edge.sublocation

        return None

    def onStart(self):
        self.current_location = None
        self.path = self.find_path_between(
            self.params["Location"].value,
            self.params["TargetLocation"].value,
        )
        return True

    def execute(self):
        if self.current_location and self.current_location.id != self.params["Location"].value.id:
            return self.step(f"Current location is not {self.current_location.label}")

        if not self.path:
            return self.success("Path has been completed")

        self.current_location = self.path[0]
        self.path = self.path[1:]
        self.params["IntermediateLocation"].value = self.current_location
        return self.step(f"Door '{self.current_location.label}' selected")


class location_is_door(PrimitiveBase):
    def createDescription(self):
        self.setDescription(LocationIsDoor(), "Location is Door")

    def execute(self):
        location = self.params["Location"].value

        is_door = location.type == "skiros:Door"
        is_open = location.getProperty("skiros:Open").value if is_door else False

        result = is_door and (is_open == self.params["Open"].value)
        if self.params["ReverseResult"].value:
            result = not result

        data = dict(location=location.label, is_door=is_door, is_open=is_open)
        if result:
            return self.success(str(data))
        else:
            return self.fail(str(data), -1)
