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
        package = self.params["Package"].value
        file = self.params["File"].value

        package_path = pkg.get_prefix_path(package)
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
