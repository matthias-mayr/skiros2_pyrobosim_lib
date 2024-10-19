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

For this workshop we have created a skill which can extract a PDDL goal from a file (with very rudimentary parsing) for planning. Try out this skill by attempting to fill in the blanks in the [planning](../planning) folder. Note that the goal of each PDDL file must abide by the following rules:

* The goal starts with _exactly_ '(:goal (and' and nothing else
* The goal ends with '))'
* There may only exist one condition per line
* Each condition starts with '('

## Problems

In this section we will assume that you have completed the skills in [1 Problem 1 Object Fetch.md](1_Problem_1_Object_Fetch.md), [2 Problem 2 Waste Disposal.md](2_Problem_2_Waste_Disposal.md), [3 Problem 3 Table Setting.md](3_Problem_3_Table_Setting.md) and [4 Problem 4 Table Waste Charge](4_Problem_4_Table_Waste_Charge.md). If you have not you can still solve these problems but you might be interested in passing `load_only_solutions:=True` to the program.

### Problem 1

For these problems you will be editing PDDL files, for problem 1 this is [src/planning/problem1.pddl](../planning/problem1.pddl).

If you formulate the condition and the planner is able to find a valid plan you might notice that the planner quite happily uses `Problem 1 - Fetch Item` which is explicitly written to solve this problem. To see if the planner is able to solve this problem without this skill you could mark it as unavailable for planning by adding `self.setAvailableForPlanning(False)` as the first line in `def createDescription(self):` in `problem_1` in [skiros2_pyrobosim_lib/problem_1_fetch_item.py](../skiros2_pyrobosim_lib/problem_1_fetch_item.py).

### Problem 2

The unfinished goal for problem 2 can be found here [src/planning/problem2.pddl](../planning/problem2.pddl).

When writing a skill which solves this problem it is slightly harder to write something which grabs all available pieces of waste compared to sending a goal which expresses that all pieces of waste need to be in the dumpster. Here the planner is able to use explicit knowledge about the state of the world as it plans while the skill should optimally be agnostic to this since it needs to work in any situation.

### Problem 3

The unfinished goal for problem 3 can be found here [src/planning/problem3.pddl](../planning/problem3.pddl).

In this problem the actions that the robot performs have become fallible, i.e. they could fail. The planner does not need to take this into account since this is handled on the skill side which means that, to the planner, this problem is very similar to problem 1, for example.

### Problem 4

For problem 4 you need to complete two files [src/planning/problem4.1.pddl](../planning/problem4.1.pddl) and [src/planning/problem4.2.pddl](../planning/problem4.2.pddl).

In this case we need to make sure that there is a valid path to the charger which does not require opening any doors before we progress with solving the rest of the problem. This can most likely be solved with a single goal depending on how you model the problem and how you express the goal but an simple pragmatic approach is to separate the problem solution into a two-stage plan. The first stage of the plan ensures that the charger is reachable and the second stage tells the planner the end state we would like to achieve.

### Summary

As a takeway we can see that a planner can simplify skill writing since it might be simpler to express something as a PDDL goal than explicitly accounting for all edge cases such that a skill can be run in any situation. Matching the flexibility of a planner is a bit harder on the skill level since the planner, by construction, is able to use knowledge about the specific state of the world.

At the same time skills allow you to hide complexity from the planner, the planner might be entirely blind to some part of the world (like unreliable execution or charging the robot) since that is entirely taken care of by the building blocks that the planner uses. By choosing an appropriate seam you are able to take advantage of the flexiblity of a planner as well as the reactive nature of behavior trees to effortlessly coordinate complex behavior.