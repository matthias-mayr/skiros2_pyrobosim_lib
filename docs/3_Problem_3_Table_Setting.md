# Problem 3 - Table Setting
**Recap**:
* In [0_Start.md](0_Start.md) we learned about the primitive skills and compound skills
* In [1 Problem 1 Object Fetch.md](1_Problem_1_Object_Fetch.md) we learned how to build a behavior tree (BT) from multiple skills.
* In [2 Problem 2 Waste Disposal.md](2_Problem_2_Waste_Disposal.md) we opened doors and moved waste into a dumpster

In this problem, we are setting the table with items from the kitchen:

| Goal | Initial State | Needed SkiROS2 Skills | Notes |
|------|---------------|------------|-------|
| 1. Bring bread and butter to the dining table.<br>2. Fridge and pantry should be closed at the end. | - Bread is in the pantry, which is closed.<br>- Butter is in the fridge, which is closed. | - Problem1<br>- Navigate<br>- CloseLocation | pyrobosim actions might fail in this and the next world  |

So we have two problems to tackle:  
1. pyrobosim actions like "pick" might fail with some probability
2. a logic to set the table

It would be annoying to develop a skill to set the table if actions might randomly fail, so let's tackle resilient action executions first.

Note that in this world we do not need to open doors to navigate between the kitchen and the dining room.

## 3.1 Action Execution that can handle Failures 

Often we want a task to fail if a single action in the execution fails. E.g. if a robot arm collides with the table while picking up an object.

Here we want to retry actions that failed and we will handle that in the skills that are in [skiros2_pyrobosim_lib/problem_3_table_setting.py](../skiros2_pyrobosim_lib/problem_3_table_setting.py).  
If we take a look at our `navigate` skills:
```python
class navigate_with_retry(SkillBase):
    def createDescription(self):
        self.setDescription(Navigate(), "Navigate to Location with Retry")

    def expand(self, skill):
        skill(
            # FIXME 3.1: Make this 'NavigateExecution' skill retryable
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

## 3.2 & 3.3 Skills for Setting the Table

Now that we have a resilient action execution, we can focus on setting the table.
> If you haven't been able to solve the previous subtask, go to [skiros2_pyrobosim_lib/solutions.py](../skiros2_pyrobosim_lib/solutions.py) and copy the content of the `expand` function of the skills between `problem_3_solution` and `problem_4_solution`. The skills you want to take the content from are called e.g. `navigate_with_retry_solution` and the content of the expand function needs to go into your `navigate_with_retry`.

Now we will need to fetch the bread and butter from the kitchen and place them on the dining table. We will also need to close the fridge and pantry at the end.

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
            # FIXME 3.2: We will fetch the bread here and later the butter
            self.skill(SerialStar())(
                # As an example, we reuse our Problem1 skill to fetch an item. The Problem1 skill has parameter 'Object' and 'ObjectTargetLocation' that we need to set. If you check the Problem3 skill description you will see that we have parameters with names like "Bread" and "Table" that we need to remap like this:
                self.skill("Problem1", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
                # FIXME 3.2: Now add skills to close the pantry


                # Unset some blackboard parameters to avoid conflicts. Nothing to do here.
                self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
                self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
            ),
            # FIXME 3.3: Now fetch the butter and close the fridge
            self.skill(SerialStar())(
                # FIXME 3.3: Add more skills to solve this task. Try to use a previous skill
            )
        )

```
You can search the codebase for `FIXME 3` for all spots where something for problem 3 needs to be done.  
Feel free to try it from here, but you can also find some more detailed instructions below.

### 3.2 Fetch bread from the Pantry and Close the Pantry

We take a look at this piece of code:
```python
# FIXME 3.2: We will fetch the bread here and later the butter
self.skill(SerialStar())(
    # As an example, we reuse our Problem1 skill to fetch an item. The Problem1 skill has parameter 'Object' and 'ObjectTargetLocation' that we need to set. If you check the Problem3 skill description you will see that we have parameters with names like "Bread" and "Table" that we need to remap like this:
    self.skill("Problem1", "", remap={"ObjectTargetLocation": "Table", "Object": "Bread"}),
    # FIXME 3.2: Now add skills to close the pantry


    # Unset some blackboard parameters to avoid conflicts. Nothing to do here.
    self.skill("BbUnsetParam", "", remap={"Parameter": "StartLocation"}),
    self.skill("BbUnsetParam", "", remap={"Parameter": "ObjectStartLocation"}),
    self.skill("BbUnsetParam", "", remap={"Parameter": "Container"}),
),
```

To keep things easy it uses the `SerialStar` processor to execute the skills in sequence and remember which one we executed. The given code already reuses the Problem1 skill that we made before. We need to remap the parameters `ObjectTargetLocation` and `Object` to the values `Table` and `Bread`.

This remapping works, because for example the parameter `ObjectTargetLocation` in the `Problem1` skill is a generic parameter that expects a `skiros:Location` and the `Table` parameter that we have here is a more specific type of location. The same with the parameters `Object` and `Bread`: bread is a specific kind of object `skiros:Part`.

Now we need to add the skills to close the pantry. So first we need to navigate to the pantry and then close it. The closing of the pantry is done with the `CloseLocation` skill. Try adding those and think about the parameters and remaps.

### 3.3 Fetch the Butter from the Fridge and Close the Fridge

Now we have this last bit remaining:
```python
# FIXME 3.3: Now fetch the butter and close the fridge
self.skill(SerialStar())(
    # FIXME 3.3: Add more skills to solve this task. Try to use a previous skill
)
```

Luckily with our skills, getting butter from the fridge is pretty much the same as getting bread from the pantry. So we can reuse the `Problem1` skill again. We need to remap the parameters and add some skills.  

Small side note: The `BbUnsetParam` skill is not needed. It's just there for technical reason to allow for autoparameterization.

One you can successfully place those two objects on the table, you have solved the problem and can move on to problem 4.

# Next

Now that you have set the table, you can move on to the [next problem, that includes the battery level](4_Problem_4_Table_Waste_Charge.md).