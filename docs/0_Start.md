# Start

Let's start by an introduction to SkiROS2. This is best explained on the [SkiROS2 wiki](https://github.com/RVMI/skiros2/wiki). This will be a very short introduction, so you can get started with the problems.

<img width="700" alt="A figure of the SkiROS2 architecture" src="https://raw.githubusercontent.com/RVMI/skiros2/ros2/skiros2/doc/SkiROS2-Architecture.svg">

### Videos
These video introductions are also helpful. Feel free to choose depending on your time budget. The longer ones always include the content of the short ones.
| 1min Pitch  | 5min Short Introduction  | 20min ROSCon 2023  |
|---|---|---|
| <a href="https://www.youtube.com/watch?v=0ejGWLx94a8"><img width="300" alt="Screenshot of the 1 minute introduction" src="https://github.com/RVMI/skiros2/blob/ros2/skiros2/res/screenshot_1min.png?raw=true"></a>  | <a href="https://www.youtube.com/watch?v=jy-LlNn3e58"><img width="300" alt="Screenshot of the 5 minute introduction" src="https://github.com/RVMI/skiros2/blob/ros2/skiros2/res/screenshot_5min.png?raw=true"></a>  | <a href="https://vimeo.com/879001825/2a0e9d5412"><img width="300" alt="Screenshot of the ROSCon 2023 talk" src="https://github.com/RVMI/skiros2/blob/ros2/skiros2/res/screenshot_roscon.png?raw=true"></a>  |
| [1min URL](https://www.youtube.com/watch?v=0ejGWLx94a8)  | [5min URL](https://www.youtube.com/watch?v=jy-LlNn3e58)  | [ROSCon Talk URL](https://vimeo.com/879001825/2a0e9d5412)  |

## Components of SkiROS2

SkiROS2 consists of three main components:
1. __Skill Manager:__ Loads and runs the skills
2. __World Model:__ The semantic RDF database to store information about the environment  
    - __Ontology__: Describes schemas for the world model, e.g. 'what is a room'
    - __Scene__: Contains instances of the ontology, e.g. 'room1 is a room'
3. __Task Manager:__ Plans tasks based on a planning goal

--> [More in the SkiROS2 Wiki introduction.](https://github.com/RVMI/skiros2/wiki)  

## Skill Model

The SkiROS2 skill model is introduced in the SkiROS2 wiki in [Overview 1 - Terminology.](https://github.com/RVMI/skiros2/wiki/Overview-1:-Terminology)  
The page [Overview 3 - Skill Model](https://github.com/RVMI/skiros2/wiki/Overview-3:-Skill-model) is the go-to page for a more detailed explanation.

<img width="700" alt="The SkiROS2 Skill Model" src="https://github.com/RVMI/skiros2/wiki/imgs/skill.svg">

tl;dr: In SkiROS2 skills consist of:
1. A __skill description__ that defines:
    - the parameters and the
    - pre-, hold- and post-conditions
2. __Skill implementations__ of these skill descriptions. These implementations can be
    - __Primitive Skills__: The lowest level of skills that interact with the robot directly in python
    - __Compound Skills__: Skills that consist of other skills and are modeled in a behavior tree (BT)

### Primitive Skill

Let's have a look at a primitive skill that interacts with `pyrobosim` to move the robot to a new location. These code snippets come from `src/skiros2_pyrobosim_lib/primitive_skills.py`.

#### **Skill Description**
This is the skill description for the navigate execution skill:
```python
class NavigateExecution(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
```
It has one parameter, that has the name `TargetLocation`. It accepts world-model elements of the type `skiros:Location` and this is a required parameter.

So what is a `skiros:Location`? It is a concept that is described in the SkiROS2 ontology in the file `skiros.owl`.  
Here in this pyrobosim world description, we have defined some subclasses of `Location`. Looking at `robi_robot_description.owl` we see these classes and subclasses:
```
Location
├── Table
├── Charger
├── Room
└── OpenableLocation
    ├── Dumpster
    ├── Pantry
    ├── Fridge
    └── Door
```
So a Table is a subclass of Location. And `OpenableLocation` even has more subclasses.  
So our `NavigateExecution` skill allows us to go to every `Location` that exists in the world model.

We can look at another short example. The `OpenExecution` skill description:
```python
class OpenExecution(SkillDescription):
    def createDescription(self):
        #=======Params=========
        self.addParam("Object", Element("skiros:OpenableLocation"), ParamTypes.Required)
```

It only allows `OpenableLocation` type world model elements as input, so e.g. a door, the fridge, etc. But not a room or a charger, because it is not possible to open those.


Feel free to try both the `Navigate Execution` and the `Open Execution` skill in the GUI to get an understanding. You will see that they allow you to choose different parameters.

#### Primitive Skill Implementation:
Our implementation for the pyrobosim navigate skill looks like this:

```python
class navigate_execution(pyrobosim_action_client_base):
    def createDescription(self):
        """Set the primitive type"""
        self.setDescription(NavigateExecution(), "Navigate Execution")

    def buildGoal(self):
        return self.pyrobosimTaskAction(ACTIONS.NAVIGATE, self.params["TargetLocation"].value.label)
```

We extracted some boilerplate code into a base class `pyrobosim_action_client_base`, which is not really relevant right now. In the `buildGoal` function, we build an action for pyrobosim to tell it where the robot should go.

What is important is that when we make our skill implementation, we set a skill description that we want to implement. In this case the `NavigateExecution` description we saw before with the parameter `TargetLocation`.

Inside the skill implementation, we can access the value of the parameter `TargetLocation` that was given to the skill while executing with:
```python
self.params["TargetLocation"].value
```

### Compound Skills

SkiROS2 uses behavior trees to combine skills to more complex ones. You can find the following snippets in `src/skiros2_pyrobosim_lib/pyrobosim_compound_skills.py`.

Let's take a look at a simplified `Navigate` skill description:
```python
class Navigate(SkillDescription):
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("skiros:Location"), ParamTypes.Required)
        self.addParam("StartLocation", Element("skiros:Location"), ParamTypes.Inferred)
        # =======PreConditions=========
        self.addPreCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "StartLocation", True))
        # =======PostConditions=========
        self.addPostCondition(self.getRelationCond("RobotAt", "skiros:at", "Robot", "TargetLocation", True))
```

Here again, we have the `TargetLocation` parameter that is required. Additionally, we have the `StartLocation` parameter that is inferred. An inferred parameter is one that we do not need to specify ourselves. So how does that work?

We also have this precondition:
```python
self.addPreCondition(ConditionRelation("RobotAt", "skiros:at", subj="Robot", obj="StartLocation", desired_state=True))
```
This can be read as:
```
Robot skiros:at StartLocation
```
and means that the robot must be at the `StartLocation` to start this skill. And here it helps us to set the `StartLocation` parameter. How?  
We already know which `Robot` we are controlling. We can just use this rule when starting the skill to see at which location the robot is at when starting to execute the skill.

#### Compound Skill Implementation

Now that we have seen the skill description of the `Navigate` skill, we can have a look at the implementation with some explanations:

```python
class navigate(SkillBase):
    # First we set a skill description the we are implementing and give it a friendly name
    def createDescription(self):
        self.setDescription(Navigate(), label="Navigate to Location")

    # Then in the `expand` function, we define the behavior tree:
    def expand(self, skill):
        # The `SerialStar` processor executes all skills in a sequence and remembers which ones have finished, so it does not execute them again.
        skill.setProcessor(SerialStar())
        # Here we build our behavior tree:
        skill(
            # First we execute our navigation
            self.skill("NavigateExecution", ""),
            # Once this succeeded. We need to update the world model with a special skill.
            # What we want to state is that we have arrived:
            # "Robot skiros:at TargetLocation"
            # Furthermore, we also need to update that we are not at the StartLocation anymore:
            self.skill("WmSetRelation", "wm_set_relation", remap={"Dst": "TargetLocation", "OldDstToRemove": "StartLocation"}, specify={'Src': self.params["Robot"].value, 'Relation': 'skiros:at', 'RelationState': True}),
    )
```

You can see a similar pattern in the other skills in `pyrobosim_compound_skills.py`.

One other short example is the skill to open locations:
```python
class open_openablelocation(SkillBase):
    def createDescription(self):
        self.setDescription(OpenOpenableLocation(), "Open Location")

    def expand(self, skill):
        skill(
            self.skill("OpenExecution", ""),
            self.skill("WmSetProperties", "",
                remap={"Src": "OpenableLocation"},
                specify={"Properties": {"skiros:Open": True}}),
        )
```
But here, after we succeeded to execute the `OpenExecution` skill, we need to set a property in the world model to remember that we just opened this `OpenableLocation`.

# Next

Now that you have an idea of the SkiROS2 skill model, you can start with the [first problem](1_Problem_1_Object_Fetch.md).