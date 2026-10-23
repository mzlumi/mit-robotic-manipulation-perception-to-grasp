<!-- Source: https://manipulation.csail.mit.edu/underactuated.html (retrieved 2026-10-27). Automatic conversion with pandoc; lossy for interactive elements, equations may need the HTML copy. -->

# Underactuated Manipulation

There is a lot more to manipulation than grasping! The term *nonprehensile manipulation* means "manipulation without grasping", and humans do it often. Pushing an object that is too big to grasp (e.g. an office chair) is a nice example, but we will see more subtle examples in this chapter, too.

For force-closure-grasping manipulation, it is possible to think of a grasped object as effectively an extension of the distal links of the robot; the geometry and inertia of the manipuland effect the kinematics and dynamics of the robot, but as a control problem the manipuland does not contribute additional degrees of freedom to the problem. Nonprehensile manipulation requires considering the additional (unactuated) degrees of freedom of the object. This makes the entire control problem (controlling the degrees of freedom of the robot + the objects) an [underactuated](http://underactuated.mit.edu/) control problem. This chapter will focus on this very interesting class of control problems in the context of manipulation.

Note that manipulation does not need to be nonprehensile to be underactuated. As a nice example, Chavan-Dafle et al. considered the case were a two fingered gripper used external environmental contacts in order to perform in-hand re-orientation (see figure below), and referred to the task as "prehensile planar pushing"Chavan-Dafle20. In this example, the grasp constrained the manipuland to the plane, but the external contacts were used to relocate and reorient the object within the grasp.

![](data/chavan-dafle-prehensile-pushing.jpg)

Prehensile planar pushing from Chavan-Dafle20; the robot exploits environmental contact to perform in-hand reorientation. Images used with permission from Chavan-Dafle. [Click here to see the animated version.](data/chavan-dafle-prehensile-pushing.gif)

# Model problems

# Planning through contact

# Visuo-motor policies
