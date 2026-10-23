<!-- Source: https://manipulation.csail.mit.edu/tactile.html (retrieved 2026-10-27). Automatic conversion with pandoc; lossy for interactive elements, equations may need the HTML copy. -->

# [Robotic Manipulation](index.html)

Perception, Planning, and Control

[Russ Tedrake](http://people.csail.mit.edu/russt/)

© Russ Tedrake, 2020-2026\
Last modified .

[How to cite these notes, use annotations, and give feedback.](misc.html)\

**Note:** These are working notes used for [a course being taught at MIT](http://manipulation.csail.mit.edu/Fall2026/). They will be updated throughout the Fall 2026 semester.

|  |  |  |
|:---|:--:|---:|
| [Previous Chapter](rl.html) | [Table of contents](index.html) | [Next Chapter](spatial.html) |

# Soft Robots and Tactile Sensing

So far in these notes, we've assumed that cameras and proprioceptive (e.g. joint) sensing were our primary sources of information about the world. There is another obvious sensor modality that we must discuss -- the sense of touch. My first goal for this chapter is to explore recent advances in sensing hardware, but also the computational frameworks that can leverage this information. (Hearing, the sense that enables the perception of sound, is likely the next most important sensing modality for humans in manipulation, but so far this field is relatively unexplored.)

Another important trend in manipulation research is the design and fabrication of robots that are fundamentally soft. Manipulation requires rich contact interactions with the environment. Most of the robots we've discussed so far are relatively very rigid; but they do always have something like rubber pads in the finger tips (the only place we traditionally expected to make contact with the environment). My second goal for this chapter is to explore advances in soft robot hardware, but also in the computational frameworks that can deal with soft robots.

These seem to be two separable ideas; so why am I putting them together into a single chapter? It turns out that being soft can enable tactile sensing. One might even argue that it's required for the richest forms of tactile sensing. Conversely, one could even argue that tactile sensing becomes the natural sensing modality for soft-skinned robots -- the natural extension of proprioception. So these two topics are intimately connected!

# Why soft?

# Soft robot hardware

# Soft-body simulation

FEM, MPM, ...

Here is the [video from a recent paper](https://www.youtube.com/watch?v=yugIJDkWR6M) describing some of the recent advances enabling high-performance, reliable FEM soft-body simulation in Drake (including interactions with rigid bodies) Han23. There is a subtle comment in the narration mentioning that the contact problem is solved to convergence on every time step -- this is in fairly stark contrast to game-engine-quality physics which cut many computational corners for the sake of performance. Amazingly we can now do this at real-time rates on a single CPU core.

# Tactile sensing

# What information do we want/need?

# Visuotactile sensing

# Whole-body sensing

One of the strongest cases in favor of whole-body tactile skins comes from the field of contact estimation. From a series of nice work, we understand fairly well how to use joint-torque sensing to extract an estimate of the location on a robot arm where contact was made. But the problem is ill-posed. Particularly in the case of multiple points of contact, joint-torque sensing alone seems woefully inadequate as a sensor Pang20a.

# Simulating tactile sensors

# Perception with tactile sensors

# Control with tactile sensors

# References

1.  Xuchen Han and Joseph Masterjohn and Alejandro Castro, "A Convex Formulation of Frictional Contact between Rigid and Deformable Bodies", arXiv preprint arXiv:2303.08912, 2023.
    \
2.  Tao Pang and Jack Umenberger and Russ Tedrake, "Identifying External Contacts from Joint Torque Measurements on Serial Robotic Arms and Its Limitations", Under Review., May, 2021. \[ [link](http://groups.csail.mit.edu/robotics-center/public_papers/Pang20a.pdf) \]
    \

|  |  |  |
|:---|:--:|---:|
| [Previous Chapter](rl.html) | [Table of contents](index.html) | [Next Chapter](spatial.html) |

------------------------------------------------------------------------

|                                                 |                      |
|-------------------------------------------------|---------------------:|
| [Accessibility](https://accessibility.mit.edu/) | © Russ Tedrake, 2026 |
