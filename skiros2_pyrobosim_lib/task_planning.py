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

        line_number = 1
        goal = []
        with open(file_path, "r") as f:
            line = ""
            while not line.strip().replace(' ', '').startswith("(:goal(and"):
                line = f.readline()
                if not line:
                    return self.fail("Unexpected EOF, did not find start of goal '(:goal (and'", -1)
                line = line.strip()
                line_number += 1

            if not line.startswith("(:goal (and"):
                return self.fail(f"Error on line {line_number}: Malformed goal does not start with (exactly) '(:goal (:and'", -1)

            if line[11:].strip():
                return self.fail(f"Error on line {line_number}: start of goal contained text after '(:goal (and': '{line[11:]}'", -1)

            goal = []
            while True:
                line = f.readline()
                if not line:
                    return self.fail(f"Unexpected EOF, goal parenthesis was not closed", -1)

                line = self.get_pddl_line(line)
                if line.startswith(")"):
                    if not line.startswith("))"):
                        return self.fail(f"Error on line {line_number}: expected goal to end with '))', got '{line}'", -1)
                    break

                line_number += 1
                if not line:
                    continue

                if sum((c == "(") - (c == ")") for c in line) != 0:
                    return self.fail(f"Error on line {line_number}: contained unclosed parenthesis. Condition cannot be broken over multiple lines.", -1)

                if not line.startswith("("):
                    return self.fail(f"Error on line {line_number}: line did not start with '('", -1)

                index = 1
                paren = 1
                for c in line[1:]:
                    paren += (c == "(") - (c == ")")
                    if paren == 0:
                        break
                    index += 1

                after_first_condition = line[index + 1:]
                if after_first_condition:
                    return self.fail(f"Error on line {line_number}: line contained text after first condition '{after_first_condition}'", -1)

                goal.append(line)

        if not goal:
            return self.fail(f"Planning file '{file}' had an empty goal", -1)

        goal_str = ",".join(goal)
        log.debug(f"Found goal\n{goal_str}")

        self.params["Goal"].value = goal_str
        return self.success(f"Extracted goal from {file}")

    def get_pddl_line(self, line):
        without_comment = line.strip().split(';')[0]
        return without_comment.strip()