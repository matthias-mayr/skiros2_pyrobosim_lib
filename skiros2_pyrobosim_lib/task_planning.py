import os
import ros2pkg.api as pkg
import skiros2_common.tools.logger as log
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, SerialStar, RetryOnFail

#################################################################################
# Descriptions
#################################################################################

class PlanFromFile(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Package", "skiros2_pyrobosim_lib", ParamTypes.Optional)
        self.addParam("File", "planning/problem1.pddl", ParamTypes.Required)


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
        self.setDescription(PlanFromFile(), "Task Planning from PDDL File")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            self.skill("ExtractPddlGoalFromFile", ""),
            self.skill("TaskPlan", ""),
        )

class extract_pddl_goal_from_file(PrimitiveBase):
    def createDescription(self):
        self.setAvailableForPlanning(False)
        self.setDescription(ExtractPddlGoalFromFile(), "Extract PDDL Goal from File")
    
    def execute(self):
        package = self.params["Package"].value
        file = self.params["File"].value

        package_path = pkg.get_prefix_path(package)
        if package_path is None:
            return self.fail(f"Could not find package '{package}'", -1)

        file_path = os.path.join(package_path, "share", package, file)

        if not os.path.isfile(file_path):
            no_file_msg = (
                f"Could not find file '{file}' under package '{package}' on path '{file_path}'. "
                "If the file is new, consider rebuilding, if the folder is new consider adding it "
                "to the build process and rebuilding."
            )
            log.warn(no_file_msg)
            return self.fail(no_file_msg, -1)

        goal = []
        with open(file_path, "r") as f:
            line = ""
            while not line.startswith("(:goal"):
                line = f.readline().strip()

            goal = [self.get_pddl_line(line[6:])]

            paren = sum((c == '(') - (c == ')') for c in line)
            while paren > 0:
                line = self.get_pddl_line(f.readline())
                goal.append(line)
                paren += sum((c == '(') - (c == ')') for c in line.split(';')[0])

            last_line = goal[-1]
            goal[-1] = ")".join(last_line.split(")")[:-1])

        if not goal:
            return self.fail(f"File '{file}' did not contain a line starting with '(:goal' or there was an unclosed parenthesis.", -1)

        goal = [g for g in goal if g]
        if not goal:
            return self.fail("Planning file had an empty goal")

        goal_str = " ".join(goal)
        log.debug(f"Found goal\n{goal_str}")

        self.params["Goal"].value = goal_str
        return self.success(f"Extracted goal from {file}")

    def get_pddl_line(self, line):
        without_comment = line.strip().split(';')[0]
        return without_comment.strip()