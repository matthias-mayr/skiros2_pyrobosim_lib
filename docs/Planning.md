# Task-level Planning

SkiROS2 has task-level planning features. This means that we can plan a sequence of skills to achieve a goal. The task planner is based on the PDDL planner `Temporal Fast Downward`.

## Background

PDDL (Planning Domain Definition Language) is a family of languages meant to standardize planning domain and problem description. A domain expressed in PDDL will contain all the available actions with their pre- and post-conditions as well as the world-model expressed in a way very similar to the SkiROS2 ontology. This makes PDDL well suited for integration in SkiROS via task planning.

Many planners have been developed some with more features than others and PDDL as a language has changed since it was made in 1998. For this reason there is a small subset of features which almost all available planners will support, like using `and`, `or` and `forall` in goal conditions. While there are less commonly supported features like `foreach` and `true-negation`.

## Planning with SkiROS2

### Domain and Problem Generation

SkiROS exposes an interface which generates a set of available actions in a PDDL file from the skills available for planning. By default a skill must define both pre- and post-conditions to be available for planning. The world model will also be translated into valid PDDL and the user will only need to supply a goal which will be given to the planner.

If one defines a skill with pre- and post-conditions yet still does not want to use it for planning one can use the  `setAvailableForPlanning` method when defining the skill description of a skill.

### Plan Execution

Once a plan has been derived SkiROS will transform it into a BT and start executing. This is essentially the control one has over the execution. Specify a goal and see if the robot is able to execute it correctly. If a plan is not found or the robot executes something unexpected one might have to tweak the conditions of the available skills. In this case a plan which solves the goal is possible to find and execute as long as it is expressed correctly.

### Goals as Strings

By default SkiROS2 supplies a skill, `Task Planning from PDDL Goal`, where one can write an arbitrary string in the SkiROS2 interface as input for planning. This skill is available in the SkiROS interface if you want to try out simple goals.

### Goals in PDDL files

For this workshop we have created a skill which can extract a PDDL goal from a file (with very rudimentary parsing) for planning. Try out this skill by attempting to fill in the blanks in the [planning](../planning) folder.