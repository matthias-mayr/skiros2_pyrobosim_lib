# Problem 3 - Table Setting
**Recap**:
* In [0_Start.md](0_Start.md) we learned about the primitive skills and compound skills
* In [1 Problem 1 Object Fetch.md](1_Problem_1_Object_Fetch.md) we learned how to build a behavior tree (BT) from multiple skills.
* In [2 Problem 2 Waste Disposal.md](2_Problem_2_Waste_Disposal.md) we opened doors and moved waste into a dumpster

In this problem, we are setting the table with items from the kitchen:

| Goal | Initial State | SkiROS2 Skills | Notes |
|------|---------------|------------|-------|
| 1. Bring bread and butter to the dining table.<br>2. Fridge and pantry should be closed at the end.. | 1. Bread is in the pantry, which is closed.<br>Butter is in the fridge, which is closed. | - Problem1<br>- Navigate<br>- CloseLocation | pyrobosim actions might fail in this and the next world  |

So we have two problems to tackle:  
1. pyrobosim actions like "pick" might fail with some probability
2. a logic to set the table

It would be annoying to develop a skill to set the table if actions might randomly fail, so let's tackle resilient action executions first.


## 1. Action Execution that can handle Failures

Often we want a task to fail if a single action in the execution fails. E.g. if a robot arm collides with the table while picking up an object.

Here we want to retry actions that failed and we will handle that in the skills that are in [skiros2_pyrobosim_lib/problem_3_table_setting.py](../skiros2_pyrobosim_lib/problem_3_table_setting.py).  
If we take a look at our `navigate` skills:
```python
class navigate_with_retry(SkillBase):
    def createDescription(self):
        self.setDescription(Navigate(), "Navigate to Location with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3: Make this 'NavigateExecution' skill retryable
            self.skill("NavigateExecution", ""),
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "TargetLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
        )
```
The default processor in SkiROS2 is `SerialStar`, so all children are executed in sequence with memory and if one child fails, the whole skills fails. Here this means that we execute `NavigateExecution` and *iff* that succeeds `WmSetRelation`. Now in this world, compared to before, `NavigateExecution` might fail and we need to handle that.

How the return signal from a skill makes its way through a behavior tree is decided by control flow nodes. In SkiROS2 uses the name processors for them and they are defined in `processors.py`. We have the following ones available:

* **Serial (->)**: The default processor. Process children in sequence until all succeed. Restarts finished (succeeded/failed) skills. Returns on first occurrence of a running of failed skill.
* **SerialStar (->*)**: Like serial, but keeps memory of which nodes previously succeeded and does not tick them again. The memory index is reset on failure, success or preemption.
* **Selector (?)**: Process children in sequence until one succeeds, ignoring failures. Returns on first occurrence of a running or successful skill.
* **SelectorStar (?*)**: Like Selector, but keeps memory of which nodes previously succeeded and do not tick them again. The memory index is reset on failure, success or preemption.
* **ParallelFf (Parallel First Fail)**: Process children in parallel until all succeed. Stop all processes if a child fails.
* **ParallelFs (Parallel First Stop)**: Process children in parallel until one succeeds. Stop all processes if a child finishes (succeeded/fail).
* **RetryOnFail**: Like Sequential, but retries up to 'max_retries' if a child node fails. Restarts all child nodes on failure.

Quite obviously we want to use the `RetryOnFail` processor for our skills here. So we want to wrap the `NavigateExecution` skill in a `RetryOnFail` processor. This processor also takes an argument on how often it should retry and we will set it to 10 retries.

Once you have adapted this skill and the remaining skills in [skiros2_pyrobosim_lib/problem_3_table_setting.py](../skiros2_pyrobosim_lib/problem_3_table_setting.py), you can test it in the pyrobosim world. The navigation should only fail if a door is blocking the path.

## 2. Skills for Setting the Table

Now that we have a resilient action execution, we can focus on setting the table. We will need to fetch the bread and butter from the kitchen and place them on the dining table. We will also need to close the fridge and pantry at the end.

There is already a skeleton for the `problem_3` skill in [skiros2_pyrobosim_lib/problem_3_table_setting.py](../skiros2_pyrobosim_lib/problem_3_table_setting.py):

```python
class problem_3(SkillBase):
    """
    """
    def createDescription(self):
        self.setDescription(Problem3(), "Problem 3 - Setting the Table")

    def expand(self, skill):
        skill.setProcessor(SerialStar())
        skill(
            # FIXME 3: We will fetch the bread here and later the butter
            self.skill(SerialStar())(
                # FIXME 3: Add more skills to solve this task. Try to use a previous skill.
                # If you could not solve problme 1, you can use the solution "Problem1Solution" instead of "Problem1" in the next line
                self.skill("Problem1", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
                

                # FIXME 3: Close the pantry


                # Unset some blackboard parameters to avoid conflicts
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            # FIXME 3: Now fetch the butter and close the fridge
            self.skill(SerialStar())(
                # FIXME 3: Add more skills to solve this task. Try to use a previous skill
            )
        )
```

You can search the codebase for `FIXME 3` for all spots where something for problem 3 needs to be done.

