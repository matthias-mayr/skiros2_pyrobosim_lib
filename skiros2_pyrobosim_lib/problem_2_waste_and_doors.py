from enum import Enum, auto
import skiros2_common.tools.logger as log
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, Selector, Serial
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

#################################################################################
# Descriptions
#################################################################################

class Problem2(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Dumpster", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Waste1", Element("skiros:Waste"), ParamTypes.Required)
        self.addParam("Waste2", Element("skiros:Waste"), ParamTypes.Required)


class NavigateAndOpenDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "TargetLocation", True))


class LocationIsDoor(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("Open", bool, ParamTypes.Required)
        self.addParam("ReverseResult", False, ParamTypes.Required)


class SelectDoorsToTarget(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("Location", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("IntermediateLocation", Element("skiros:Location"), ParamTypes.Optional)


#################################################################################
# Implementations
#################################################################################


class problem_2(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem2(), "Problem 2 - Waste Disposal")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            # First we need to open the dumpster, because we need an empty gripper to do that.
            # As a side-effect, this also opens all the doors leading to the dumpster.
            self.skill(SerialStar())(
                # Navigate to the dumpster with the new skill that opens doors on the way
                self.skill("Navigate", "navigate_and_open_doors", remap={"TargetLocation": "Dumpster"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                # FIXME 2.1: Add a skill to open the dumpster:

            ),
            # As a next step we need to pick up the first waste item and dispose of it
            self.skill(SerialStar())(
                # FIXME 2.2: Add a skill to pick up the first waste object and that brings it to the dumpster. Try to reuse a previously implemented skill

                # We need to clean up some parameters on the blackboard. Nothing to do here.
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            # Now we want to pick up the second waste item and dispose of it
            self.skill(SerialStar())(
                # FIXME 2.3: Add a skill to pick up the second waste object and that brings it to the dumpster. Try to reuse a previously implemented skill
            ),
            # FIXME 2.4: Add a skill to close the dumpster after disposing of the waste
            
        )

class navigate_and_open_door(SkillBase):
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(NavigateAndOpenDoor(), "Navigate and Open Single Door")
    
    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("Navigate", ""),
            self.skill(Selector())(
                self.skill("LocationIsDoor", "", remap={"Location": "TargetLocation"}, specify={"ReverseResult": True, "Open": False}),
                self.skill("OpenOpenableLocation", "", remap={"OpenableLocation": "TargetLocation"}),
            ),
        )

class navigate_and_open_doors(SkillBase):
    def createDescription(self):
        self.setDescription(NavigateAndOpenDoor(), "Navigate and Open Doors")

    def modifyDescription(self, skill):
        skill.addParam("IntermediateLocation", Element("skiros:Location"), ParamTypes.Optional)
        skill.addParam("FirstStartLocation", Element("skiros:Location"), ParamTypes.Optional)

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("BbUnsetParam", "", remap={"Parameter": "IntermediateLocation"}),
            self.skill("CopyValue", "", remap={"Input": "StartLocation", "Output": "FirstStartLocation", }),
            self.skill(Serial())(
                self.skill(Selector())(
                    self.skill("IsNone", "", remap={"Param": "IntermediateLocation"}),
                    self.skill(SerialStar())(
                        self.skill("NavigateAndOpenDoor", "navigate_and_open_door", remap={"TargetLocation": "IntermediateLocation"}),
                        self.skill("CopyValue", "", remap={"Output": "StartLocation", "Input": "IntermediateLocation"}),
                    ),
                ),
                self.skill("SelectDoorsToTarget", "", remap={"Location": "StartLocation"}),
                self.skill("CopyValue", "", remap={"Input": "FirstStartLocation", "Output": "StartLocation"}),
            ),
        )


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

        self.current_location, self.path = self.path[0], self.path[1:]
        if self.location_type(self.current_location) is self.Edge.door and not self.current_location.getProperty("skiros:Open").value:
            self.params["IntermediateLocation"].value = self.current_location
            return self.step("Door needs to be opened")
        elif not self.path:
            return self.success("Path has been completed")

        self.current_location, self.path = self.path[0], self.path[1:]
        self.params["IntermediateLocation"].value = self.current_location
        return self.step(f"Door '{self.current_location.label}' selected")
