# Problem 2 - Waste Disposal

In [0_Start.md](0_Start.md) we learned about the primitive skills and compound skills.  
In [1 Problem 1 Object Fetch.md](1_Problem_1_Object_Fetch.md) we learned how to build a behavior tree (BT) from multiple skills.

In this problem, we are going to throw waste items into a large bin:
| Goal | Initial State | SkiROS2 Skills | Notes |
|------|---------------|------------|-------|
| - Waste should be in the dumpster.<br>- Dumpster should be closed. | Waste is on the office desk and in the office bin.<br>Hallways into the trash room are closed. | - Pick<br>- Navigate<br>- Place | Closed doors can not be passed |


```python
class problem_2(SkillBase):

```

You can search the codebase for `FIXME 2` for all spots where something for problem 2 needs to be done.

