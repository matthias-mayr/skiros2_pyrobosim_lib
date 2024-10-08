import os
import ros2pkg.api
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, RetryOnFail
from skiros2_std_skills.task_planner import task_plan

#################################################################################
# Descriptions
#################################################################################

class PlanFromFile(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Package", "skiros2_pyrobosim_lib", ParamTypes.Optional)
        self.addParam("File", "pddl/test.pddl", ParamTypes.Required)


class ExtractPddlGoalFromFile(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Package", str, ParamTypes.Required)
        self.addParam("File", str, ParamTypes.Required)
        self.addParam("Goal", str, ParamTypes.Optional)

#################################################################################
# Implementations
#################################################################################

class plan_from_file(SkillBase):
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(PlanFromFile(), "Plan from File")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("ExtractPddlGoalFromFile", ""),
            self.skill(RetryOnFail(10))(
                self.skill("TaskPlan", ""),
            ),
        )

class extract_pddl_goal_from_file(PrimitiveBase):
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(ExtractPddlGoalFromFile(), "Extract PDDL Goal from File")
    
    def execute(self):
        import ros2pkg
        import skiros2_common.tools.logger as log

        package = self.params["Package"].value
        file = self.params["File"].value

        package_path = ros2pkg.api.get_prefix_path(package)
        if package_path is None:
            return self.fail(f"Could not find package '{package}'", -1)

        file_path = os.path.join(package_path, "share", package, file)

        if not os.path.isfile(file_path):
            log.warn(f"Could not find file '{file}' under package '{package}' on path '{file_path}', if the file is new, consider rebuilding")
            return self.fail(f"Could not find file '{file}' under package '{package}' on path '{file_path}', if the file is new, consider rebuilding", -1)

        goal = []
        with open(file_path, "r") as f:
            line = ""
            while not line.startswith("(:goal"):
                line = f.readline().strip()

            goal = [line[6:] + '\n']

            paren = sum((c == '(') - (c == ')') for c in line)
            while paren > 0:
                line = f.readline()
                goal.append(line)
                paren += sum((c == '(') - (c == ')') for c in line)

            last_line = goal[-1]
            goal[-1] = ")".join(last_line.split(")")[:-1])

        if not goal:
            return self.fail(f"Could not find a valid pddl goal in file '{file}'", -1)

        goal_str = "".join(goal)
        log.debug(f"Found goal\n{goal_str}")

        self.params["Goal"].value = goal_str
        return self.success(f"Extracted goal from {file}")


# class planner_solution_1(task_plan):
#     def modifyDescription(self, skill):
#         on_table = pddl_condition("skiros:contain", "skiros:Table-15", "{x}")
#         all_snacks_on_table = pddl_forall("skiros:Snacks", on_table)

#         self.addParam("Goal", all_snacks_on_table, ParamTypes.Required)


# class planner_solution_2(task_plan):
#     def modifyDescription(self, skill):
#         in_dumpster = pddl_condition("skiros:contain", "skiros:Location-20", "{x}")
#         all_in_dumpster = pddl_forall("skiros:Waste", in_dumpster)
#         dumpster_closed = pddl_property("skiros:Location-20", "skiros:Open", False)

#         self.addParam("Goal", pddl_and(all_in_dumpster, dumpster_closed), ParamTypes.Required)


# class planner_solution_3(task_plan):
#     def modifyDescription(self, skill):
#         table = "skiros:Table-15"
#         bread_on_table = pddl_condition("skiros:contain", table, "skiros:Bread-")
#         butter_on_table = pddl_condition("skiros:contain", table, "skiros:Butter-")
#         fridge_closed = pddl_property("skiros:Fridge-17", "skiros:Open", False)
#         pantry_closed = pddl_property("skiros:Pantry-18", "skiros:Open", False)

#         self.addParam("Goal", pddl_and(bread_on_table, butter_on_table, fridge_closed, pantry_closed), ParamTypes.Required)


# class planner_solution_4(task_plan):
#     def modifyDescription(self, skill):
#         # Bread and butter on dining room table, fridge and pantry closed
#         # All waste in dumpster, dumpster closed
#         # Don't run out of battery
#         in_dumpster = pddl_condition("skiros:contain", "skiros:Location-20", "{x}")
#         all_in_dumpster = pddl_forall("skiros:Waste", in_dumpster)
#         dumpster_closed = pddl_property("skiros:Location-20", "skiros:Open", False)

#         table = "skiros:Table-15"
#         bread_on_table = pddl_condition("skiros:contain", table, "skiros:Bread-")
#         butter_on_table = pddl_condition("skiros:contain", table, "skiros:Butter-")
#         fridge_closed = pddl_property("skiros:Fridge-17", "skiros:Open", False)
#         pantry_closed = pddl_property("skiros:Pantry-18", "skiros:Open", False)

#         goal = pddl_and(
#             all_in_dumpster,
#             dumpster_closed,
#             bread_on_table,
#             butter_on_table,
#             fridge_closed,
#             pantry_closed
#         )
#         self.addParam("Goal", goal, ParamTypes.Required)


# def pddl_condition(condition, from_param, to_param):
#     return f"({condition} {from_param} {to_param})"


# def pddl_property(obj, prop, value):
#     return f"(PROPERTY {obj} {prop} {value})"


# def pddl_and(*conditions):
#     return f"(and {' '.join(conditions)})"


# def pddl_forall(of_type, condition, param="x"):
#     return f"(forall (?x - {of_type}) {condition.format(**{param: "?x"})})"
