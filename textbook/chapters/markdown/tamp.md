<!-- Source: https://manipulation.csail.mit.edu/tamp.html (retrieved 2026-10-27). Automatic conversion with pandoc; lossy for interactive elements, equations may need the HTML copy. -->

# [Robotic Manipulation](index.html)

Perception, Planning, and Control

[Russ Tedrake](http://people.csail.mit.edu/russt/)

© Russ Tedrake, 2020-2026\
Last modified .

[How to cite these notes, use annotations, and give feedback.](misc.html)\

**Note:** These are working notes used for [a course being taught at MIT](http://manipulation.csail.mit.edu/Fall2026/). They will be updated throughout the Fall 2026 semester.

|     |                                 |     |
|:----|:-------------------------------:|----:|
|     | [Table of contents](index.html) |     |

# Task and Motion Planning

As we develop more and more sophisticated manipulation capabilities, we will need increasingly sophisticated programming strategies for the the high-level (aka "task-level") execution of the robot. For the relatively simple task of moving YCB objects between bins, a simple [state-machine](clutter.html#task)-programming paradigm was sufficient. If the task is "make me a pizza", where we need to find and fetch all of the tools and ingredients from various places throughout the kitchen, it would be unwieldy to program a state machine that can work in any kitchen.

In this chapter we'll look at the problem of programming the task-level using AI-style *symbolic* planning, which has advanced significantly over the last decades to the point where it can solve enormously complex problems. But to ground the symbols in our robot motions and constraints, we must tightly integrate the task-level planning with the lower-level motion planning; this integration is the object of study in *task and motion planning (TAMP)*. For an excellent review, see Garrett21.

# A case study: Loading the dishwasher

# Symbolic Planning

# STRIPS

# PDDL

# Factorized planners

Fastforward, fastdownward

# Adding continuous variables to symbolic planners

# Adding discrete variables to motion planners

# Graphs of Convex Sets

# References

1.  Caelan Reed Garrett and Rohan Chitnis and Rachel Holladay and Beomjoon Kim and Tom Silver and Leslie Pack Kaelbling and Tomás Lozano-Pérez, "Integrated task and motion planning", Annual review of control, robotics, and autonomous systems, vol. 4, pp. 265--293, 2021.
    \

|     |                                 |     |
|:----|:-------------------------------:|----:|
|     | [Table of contents](index.html) |     |

------------------------------------------------------------------------

|                                                 |                      |
|-------------------------------------------------|---------------------:|
| [Accessibility](https://accessibility.mit.edu/) | © Russ Tedrake, 2026 |
