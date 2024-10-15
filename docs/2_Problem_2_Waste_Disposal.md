# Problem 2 - Waste Disposal
**Recap**:
* In [0_Start.md](0_Start.md) we learned about the primitive skills and compound skills.  
* In [1 Problem 1 Object Fetch.md](1_Problem_1_Object_Fetch.md) we learned how to build a behavior tree (BT) from multiple skills.

In this problem, we are going to throw waste items into a large bin:
| Goal | Initial State | SkiROS2 Skills | Notes |
|------|---------------|------------|-------|
| 1. Waste should be in the dumpster.<br>2. Dumpster should be closed. | Waste is on the office desk and in the office bin.<br>Hallways into the trash room are closed. | - Problem1<br>- Close<br>- Open | Closed doors need to be opened to navigate through them |

This implementation will "manually" place the two waste items in the dumpster by moving the first and then moving the second. It would also be possible to write a more advanced implementation that moves all waste items that exist in the world.

In this world, it is important to know that the doors leading to the dumpster are closed. We need to open them before we can navigate to the dumpster. The normal navigate skill will not be able to find a path through a blocked door. For this problem, we supply a new navigation skill that can navigate and open doors. This skill allows us to specify a target location and it will open all doors on the way to that location.

So let's look at the skill skeleton in [skiros2_pyrobosim_lib/problem_2_waste_and_doors.py](../skiros2_pyrobosim_lib/problem_2_waste_and_doors.py):

```python
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
```
You can try to work out the problem with only the comments that are given in the code. If you need more help, you can find further instructions below.  
You can search the codebase for `FIXME 2` for all spots where something for problem 2 needs to be done.

## 2.1 Opening the Dumpster

We need to open the dumpster before we can dispose of the waste. This is important because we need an empty gripper to open the dumpster. It is of course possible to put down the object, open the door, and pick it up again, but for the sake of simplicity will just open the dumpster first.  
As a side effect, this also makes sure that we have a clear path to the dumpster.

Now as an actual task here, we only need to open the dumpster. You can use the "OpenLocation" skill to do that. Check out the parameters and make an appropriate remap.

## 2.2 Waste Transport

Now we need to pick up the first waste item and bring it to the dumpster. We could use "Pick", "Navigate" and "Place", but you might remember from the previous problem that we just wrote a skill to do exactly this.  
You can reuse the "Problem1" skill to pick up the first waste item and bring it to the dumpster. Check out the parameters and make an appropriate remap.

At the end of this block, some parameters need to be removed from the blackboard. In ROS 2 this is currently necessary to avoid conflicts. You can just ignore that part.

## 2.3 Waste Transport 2

Now we need to pick up the second waste item and bring it to the dumpster. You might noticee that again, we can just re-use the "Problem1" skill to do this.

## 2.4 Closing the Dumpster

As a final step we need to close the dumpster and are done.

# Next

Continue with the [next problem, the table setting problem](3_Problem_3_Table_Setting.md).