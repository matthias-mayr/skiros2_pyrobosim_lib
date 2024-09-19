###  SkiROS2 pyrobosim Library

A warm welcome to the SkiROS2 setup for the [problems of the deliberation group ROSCon24 workshop](https://github.com/ros-wg-delib/roscon24-workshop). This repository contains skills and knowledge representation for the started problem. Among the skills are wrapper skills for `pyrobosim` actions as well as sceletons to solve the problems and their solutions.

**SkiROS2** is a platform to create complex robot behaviors by composing _skills_ - modular software blocks - into [behavior trees](https://en.wikipedia.org/wiki/Behavior_tree_(artificial_intelligence,_robotics_and_control)).

Robots coordinated with SkiROS can be used in partially structured environments, where the robot has a good initial understanding of the environment, but it is also expected to find discrepancies, fail using initial plans and react accordingly.

SkiROS offers the following features:

* A **framework** to organize the **robot behaviors** within **modular skill libraries**
* Scalable skill model with **pre-, hold- and post-conditions**
* A **reactive execution engine** based on **Behavior trees**
* A **world model** as a **semantic database** to manage environmental knowledge
* **Reasoning capabilities** and **automatic inference** of skill parameters
* An integration point for **PDDL task planning**
* **Automatic generation of planning domains** based on the skills and entities in the world model
* **ROS**, **RViz** and **tf integration**
* **Python APIs** for skill handling, the world model and task planning

<img width="700" alt="A figure of the SkiROS2 architecture" src="https://raw.githubusercontent.com/RVMI/skiros2/ros2/skiros2/doc/SkiROS2-Architecture.svg">

## Getting Started with SkiROS2 as a Whole

* **A full introduction and the tutorials are located in the [SkiROS2 wiki](https://github.com/RVMI/skiros2/wiki)**
* Watch a video from the [video section below](#Videos)
* Additionally, the [SkiROS2 paper](https://arxiv.org/abs/2306.17030) provides an overview and background information

### Videos
We have video introductions to the platform with varying lengths. Feel free to choose depending on your time budget. The longer ones always include the content of the short ones.
| 1min Pitch  | 5min Short Introduction  | 20min ROSCon 2023  |
|---|---|---|
| <a href="https://www.youtube.com/watch?v=0ejGWLx94a8"><img width="300" alt="Screenshot of the 1 minute introduction" src="https://github.com/RVMI/skiros2/blob/ros2/skiros2/res/screenshot_1min.png?raw=true"></a>  | <a href="https://www.youtube.com/watch?v=jy-LlNn3e58"><img width="300" alt="Screenshot of the 5 minute introduction" src="https://github.com/RVMI/skiros2/blob/ros2/skiros2/res/screenshot_5min.png?raw=true"></a>  | <a href="https://vimeo.com/879001825/2a0e9d5412"><img width="300" alt="Screenshot of the ROSCon 2023 talk" src="https://github.com/RVMI/skiros2/blob/ros2/skiros2/res/screenshot_roscon.png?raw=true"></a>  |
| [1min URL](https://www.youtube.com/watch?v=0ejGWLx94a8)  | [5min URL](https://www.youtube.com/watch?v=jy-LlNn3e58)  | [ROSCon Talk URL](https://vimeo.com/879001825/2a0e9d5412)  |

## How to get Started with the pyrobosim Problems
You can launch SkiROS2 together with pyrobosim with this command:
```
ros2 launch skiros2_pyrobosim_lib main.launch.py problem_number:=1
```

This brings up the SkiROS2 interface as well as the pyrobosim world. As a first thing, it is a good browse through the available skills and the world model. You can do this with the SkiROS2 interface or by looking at the files in the `skiros2_pyrobosim_lib` package.


## Repository Structure

Below you'll find the structure of this repository.  
You can also get it directly from [github.com/matthias-mayr/skiros2_pyrobosim_lib](https://github.com/matthias-mayr/skiros2_pyrobosim_lib)

```
├── README.md  
├── docs                 ### The documents guiding you through 
│   ├── 0_Start.md                  --> Introduction
│   └── 1_Problem_1_Object_Fetch.md --> Instructions for Problem 1
├── launch
│   └── main.launch.py              --> Launch file for SkiROS2 in the workshop
├── owl                     ### Knowledge: scenes and ontology
│   ├── p1_scene.turtle     ├── Scenes for problem 1
│   ├── p2_scene.turtle     ├── to
│   ├── p3_scene.turtle     ├── problem
│   ├── p4_scene.turtle     └── 4
│   └── robi_robot_description.owl  --> Ontology with classes & relations
├── scripts
│   └── yaml_world_to_turtle.py     --> Converts world.yaml to scene.turtle 
└── skiros2_pyrobosim_lib           ### All our skills
    ├── basic_compound_skills.py    --> Skills wrapping pyrobosim primituves
    ├── extra_compound_skills.py    --> More advanced skills
    ├── primitive_skills.py         --> Skills for pyrobosim & python directly
    ├── reasoning_skills.py         --> Skills that use relations to reason
    ├── solutions.py                --> Don't look here until you're done:-)
    └── utils                       --> Extra utility skills
        └── blackboard.py
```

## Let's get Started

Follow this link or just go in the docs folder [docs/0_Start.md](docs/0_Start.md) or view it online [on the Github repo](https://github.com/matthias-mayr/skiros2_pyrobosim_lib/tree/main/docs/0_Start.md).