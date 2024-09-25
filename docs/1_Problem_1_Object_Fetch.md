# Problem 1

In the previous markdown file [0_Start.md](0_Start.md) we learned about the primitive skills and compound skills.

The primitive skills interface directly with `pyrobosim` are in [src/skiros2_pyrobosim_lib/primitive_skills.py](../skiros2_pyrobosim_lib/primitive_skills.py).  
The compound skills that use and update the world model (WM) are in [src/skiros2_pyrobosim_lib/basic_compound_skills.py](../skiros2_pyrobosim_lib/basic_compound_skills.py).  

Now we want to use them to solve problem 1. Navigate to [src/skiros2_pyrobosim_lib/problem_skills.py](../skiros2_pyrobosim_lib/problem_skills.py). You will find a skeleton for a compound skill to fill out:

```python
class problem_1(SkillBase):
    def createDescription(self):
        self.setDescription(description=Problem1(), label="Problem 1 - Fetch Item")

    def expand(self, skill):
        # We want to execute some skills in a sequence and remember which one we executed, so we use SerialStar. It will abort the skill if any of the child skills fail:
        skill.setProcessor(SerialStar())
        skill(
            # As a first step, we want to navigate to the location where the object is currently located. In our skill, this is saved in the 'ObjectStartLocation' parameter.
            # However, the 'Navigate' skill takes only a parameter 'TargetLocation', so we need to remap/rewire those parameters like this:
            self.skill("Navigate", "", remap={"TargetLocation": "ObjectStartLocation"}),
            # FIXME 1 - Add more skills to solve this task
            # From here, feel free to add more skills. Check out 'basic_compound_skills.py' to see which ones are available:
        )
```

You can search the codebase for `FIXME 1` for all spots where something for problem 1 needs to be done.

## Using and Updating this Skill
In order to use this skill, it needs to be in the list of skills to be loaded in `main.launch.py`. For the `problem_1` skill, this is already done.  
From the GUI you can just select the skill and run it:



After doing any changes in code, the skill needs to be loaded with the skill manager. You can either launch the whole launch file from scratch or especially in debug mode, just restart the skill manager node.

