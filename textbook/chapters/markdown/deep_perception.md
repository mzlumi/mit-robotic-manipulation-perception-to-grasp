<!-- Source: https://manipulation.csail.mit.edu/deep_perception.html (retrieved 2026-10-27). Automatic conversion with pandoc; lossy for interactive elements, equations may need the HTML copy. -->

# [Robotic Manipulation](index.html)

Perception, Planning, and Control

[Russ Tedrake](http://people.csail.mit.edu/russt/)

© Russ Tedrake, 2020-2026\
Last modified .

[How to cite these notes, use annotations, and give feedback.](misc.html)\

**Note:** These are working notes used for [a course being taught at MIT](http://manipulation.csail.mit.edu/Fall2026/). They will be updated throughout the Fall 2026 semester.

|  |  |  |
|:---|:--:|---:|
| [Previous Chapter](segmentation.html) | [Table of contents](index.html) | [Next Chapter](rl.html) |

# Deep Perception for Manipulation

In the previous chapter, we discussed deep-learning approaches to object detection and (instance-level) segmentation; these are general-purpose tasks for processing RGB images that are used broadly in computer vision. Detection and segmentation alone can be combined with geometric perception to, for instance, estimate the pose of a known object in just the segmented point cloud instead of the entire scene, or to run our point-cloud grasp selection algorithm only on the segmented point cloud in order to pick up an object of interest.

One of the most amazing features of deep learning for perception is that we can pre-train on a different dataset (like ImageNet or COCO) or even a different task and then fine-tune on our domain-specific dataset or task. But what are the right perception tasks for manipulation? Object detection and segmentation are a great start, but often we want to know more about the scene/objects to manipulate them. That is the topic of this chapter.

There is a potential answer to this question that we will defer to a later chapter: learning end-to-end "visuomotor" policies, sometimes affectionately referred to as "pixels to torques". Here I want us to think first about how we can combine a deep-learning-based perception system with the powerful existing (model-based) tools that we have been building up for planning and control.

maybe a system diagram which includes perception, planning and control? So far we’ve had two version - grasp candidates and/or object pose…

I'll start with the deep-learning version of a perception task we've already considered: object pose estimation.

# Pose estimation

We discussed pose estimation at some length in the [geometric perception chapter](pose.html), and had a few take-away messages. Most importantly, the geometric approaches have only a very limited ability to make use of RGB values; but these are incredibly valuable for resolving a pose. Geometry alone doesn't tell the full story. Another subtle lesson was that the ICP loss, although conceptually very clean, does not sufficiently capture the richer concepts like non-penetration and free-space constraints. As the original core problems in 2D computer vision started to feel "solved", we've seen a surge of interest/activity from the computer vision community on 3D perception, which is great for robotics!

The conceptually simplest version of this problem is that we would like to estimate the pose of a known object from a single RGB image. How should we train a mustard-bottle-specific (for example) deep network which takes an RGB image in, and outputs a pose estimate? Of course, if we can do this, we can also apply the idea to e.g. the images cropped from the bounding box output of an object recognition / instance segmentation system.

# Pose representation

Once again, we must confront the question of how best to represent a pose. Many initial architectures discretized the pose space into bins and formulated pose estimation as a classification problem, but the trend eventually shifted towards pose regressionMahendran17+Xiang17. Regressing three numbers to represent x-y-z translation seems clear, but we have many choices for how to represent 3D orientation (e.g. Euler angles, unit quaternions, rotation matrices, ...), and our choice can impact learning and generalization performance.

To output a single pose, works like Zhou19 and Levinson20 argue that many rotation parameterizations have issues with discontinuities. They recommend having the network output 6 numbers for the rotation (and then projecting to SO(3) via Gram-Schmidt orthogonalization), or outputting the full 9 numbers of the rotation matrix, and projecting back to SO(3) via [SVD orthogonalization](pose.html#registration).

Perhaps more substantially, many works have pointed out that outputting a single "correct" pose is simply not sufficient Hashimoto20+Deng22. When objects have rotational symmetries, or if they are severely occluded, then outputting an entire pose distribution is much more adequate. Representing a categorial distribution is very natural for discretized representations, but how do we represent distributions over continuous pose? One very elegant choice is the Bingham distribution, which gives the natural generalization of Gaussians applied to unit quaternions Glover14+Peretroukhin20+Deng22.

# Loss functions

No matter which pose representation is coming out of the network and the ground truth labels, one must choose the appropriate loss function. Quaternion-based loss functions can be used to compute the geodesic distance between two orientations, and should certainly be more appropriate than e.g. a least-squares metric on Euler angles. More expensive, but potentially more suitable is to write the loss function in terms of a reconstruction error so that the network is not artificially penalized for e.g. symmetries which it could not possibly address Hodan20.

Training a network to output an entire distribution over pose brings up additional interesting questions about the choice for the loss function. While it is possible to train the distribution based on only the statistics of the data labeled with ground truth pose (again, choosing maximum likelihood loss vs mean-squared error), it is also possible to use our understanding of the symmetries to provide more direct supervision. For example, Hashimoto20 used image differences to efficiently (but heuristically) estimate a ground-truth covariance for each sample.

# Pose estimation benchmarks

Benchmark for 6D Object Pose Estimation (BOP)Hodan20.

# Limitations

Although pose estimation is a natural task, and it is straightforward to plug an estimated pose into many of our robotics pipelines, I feel pretty strongly that this is often not the right choice for connecting perception to planning and control. Although some attempts have been made to generalize pose to categories of objects Wang19, pose estimation is pretty strongly tied to known objects, which feels limiting. Accurately estimating the pose of an object is difficult, and is often not necessary for manipulation.

# Grasp selection

In Gibson77+Gibson79, J.J. Gibson articulated his highly influential *theory of affordances*, where he described the role of perception as serving behavior (and behavior being controlled by perception). In our case, affordances describe not what the object/environment is nor its pose, but what opportunities for action it provides to the robot. Importantly, one can potentially estimate affordances without explicitly estimating pose.

Two of the earliest and most successful examples of this were tenPas17+tenPas18 and Mahler17+Mahler17a, which attempted to learn a grasping affordances directly from raw RGB-D input. tenPas17 used a grasp selection strategy that was very similar to (and partly the inspiration for) the [grasp sampler](clutter.html#grasp_sampler) we introduced in our antipodal grasping workflow. But in addition to the geometric heuristic, tenPas17 trained a small CNN to predict grasp success. The input to this CNN was a voxelized summary of the pixels and local geometry estimates in the immediate vicinity of the grasp, and the output was a binary prediction of grasp success or failure. Similarly, Mahler17 learned a "grasp quality" CNN takes as input the depth image in the grasp frame (+ the grasp depth in the camera frame) and outputs a probability of grasp success.

Transporter nets Zeng21

# (Semantic) Keypoints

KeyPoint Affordances for Category-Level Robotic Manipulation (kPAM) Manuelli19+Gao20b.

Example: corner keypoints for boxes. (also pose+shape estimation from keypoints?)

# Dense Correspondences

Florence18a, DynoV2, ...

# Scene Flow

https://proceedings.mlr.press/v205/seita23a/seita23a.pdf and other work by Held should get me started.

# Task-level state

# Other perceptual tasks / representations

My coverage above is necessarily incomplete and the field is moving fast. Here is a quick "shout out" to a few other very relevant ideas.

More coming soon...

# Exercises

# Deep Object Net and Contrastive Loss

In this problem you will further explore Dense Object Nets, which were introduced in lecture. Dense Object Nets are able to quickly learn consistent pixel-level representations for visual understanding and manipulation. Dense Object Nets are powerful because the representations they predict are applicable to both rigid and non-rigid objects. They can also generalize to new objects in the same class and can be trained with self-supervised learning. For this problem you will work in

to first implement the loss function used to train Dense Object Nets, and then predict correspondences between images using a trained Dense Object Net.

# References

1.  Siddharth Mahendran and Haider Ali and René Vidal, "3d pose regression using convolutional neural networks", Proceedings of the IEEE International Conference on Computer Vision Workshops , pp. 2174--2182, 2017.
    \
2.  Yu Xiang and Tanner Schmidt and Venkatraman Narayanan and Dieter Fox, "Posecnn: A convolutional neural network for 6d object pose estimation in cluttered scenes", arXiv preprint arXiv:1711.00199, 2017.
    \
3.  Yi Zhou and Connelly Barnes and Jingwan Lu and Jimei Yang and Hao Li, "On the continuity of rotation representations in neural networks", Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition , pp. 5745--5753, 2019.
    \
4.  Jake Levinson and Carlos Esteves and Kefan Chen and Noah Snavely and Angjoo Kanazawa and Afshin Rostamizadeh and Ameesh Makadia, "An analysis of svd for deep rotation estimation", Advances in Neural Information Processing Systems, vol. 33, pp. 22554--22565, 2020.
    \
5.  Kunimatsu Hashimoto\* and Duy-Nguyen Ta\* and Eric Cousineau and Russ Tedrake, "KOSNet: A Unified Keypoint, Orientation and Scale Network for Probabilistic 6D Pose Estimation", Under review , 2020. \[ [link](http://groups.csail.mit.edu/robotics-center/public_papers/Hashimoto20.pdf) \]
    \
6.  Haowen Deng and Mai Bui and Nassir Navab and Leonidas Guibas and Slobodan Ilic and Tolga Birdal, "Deep bingham networks: Dealing with uncertainty and ambiguity in pose estimation", International Journal of Computer Vision, vol. 130, no. 7, pp. 1627--1654, 2022.
    \
7.  Jared Marshall Glover, "The Quaternion Bingham Distribution, 3D Object Detection, and Dynamic Manipulation", PhD thesis, Massachusetts Institute of Technology, May, 2014.
    \
8.  Valentin Peretroukhin and Matthew Giamou and David M Rosen and W Nicholas Greene and Nicholas Roy and Jonathan Kelly, "A smooth representation of belief over so (3) for deep rotation learning with uncertainty", arXiv preprint arXiv:2006.01031, 2020.
    \
9.  Tomás Hodan and Martin Sundermeyer and Bertram Drost and Yann Labbé and Eric Brachmann and Frank Michel and Carsten Rother and Jirí Matas, "BOP Challenge 2020 on 6D Object Localization", European Conference on Computer Vision Workshops (ECCVW), 2020.
    \
10. He Wang and Srinath Sridhar and Jingwei Huang and Julien Valentin and Shuran Song and Leonidas J Guibas, "Normalized object coordinate space for category-level 6d object pose and size estimation", Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition , pp. 2642--2651, 2019.
    \
11. James J Gibson, "The theory of affordances", Hilldale, USA, vol. 1, no. 2, pp. 67--82, 1977.
    \
12. James J Gibson, "The ecological approach to visual perception", Psychology press , 1979.
    \
13. Andreas ten Pas and Marcus Gualtieri and Kate Saenko and Robert Platt, "Grasp pose detection in point clouds", The International Journal of Robotics Research, vol. 36, no. 13-14, pp. 1455--1473, 2017.
    \
14. Andreas Ten Pas and Robert Platt, "Using geometry to detect grasp poses in 3d point clouds", Robotics Research: Volume 1, pp. 307--324, 2018.
    \
15. Jeffrey Mahler and Jacky Liang and Sherdil Niyaz and Michael Laskey and Richard Doan and Xinyu Liu and Juan Aparicio Ojea and Ken Goldberg, "Dex-net 2.0: Deep learning to plan robust grasps with synthetic point clouds and analytic grasp metrics", arXiv preprint arXiv:1703.09312, 2017.
    \
16. Jeffrey Mahler and Ken Goldberg, "Learning deep policies for robot bin picking by simulating robust grasping sequences", Conference on robot learning , pp. 515--524, 2017.
    \
17. Andy Zeng and Pete Florence and Jonathan Tompson and Stefan Welker and Jonathan Chien and Maria Attarian and Travis Armstrong and Ivan Krasin and Dan Duong and Vikas Sindhwani and others, "Transporter networks: Rearranging the visual world for robotic manipulation", Conference on Robot Learning , pp. 726--747, 2021.
    \
18. Lucas Manuelli\* and Wei Gao\* and Peter Florence and Russ Tedrake, "kPAM: KeyPoint Affordances for Category-Level Robotic Manipulation", arXiv e-prints, pp. arXiv:1903.06684, Mar, 2019. \[ [link](https://sites.google.com/view/kpam) \]
    \
19. Wei Gao and Russ Tedrake, "kPAM 2.0: Feedback control for generalizable manipulation", IEEE Robotics and Automation Letters, 2020. \[ [link](http://groups.csail.mit.edu/robotics-center/public_papers/Gao20b.pdf) \]
    \
20. Peter R. Florence\* and Lucas Manuelli\* and Russ Tedrake, "Dense Object Nets: Learning Dense Visual Object Descriptors By and For Robotic Manipulation", Conference on Robot Learning (CoRL) , October, 2018. \[ [link](http://groups.csail.mit.edu/robotics-center/public_papers/Florence18a.pdf) \]
    \

|  |  |  |
|:---|:--:|---:|
| [Previous Chapter](segmentation.html) | [Table of contents](index.html) | [Next Chapter](rl.html) |

------------------------------------------------------------------------

|                                                 |                      |
|-------------------------------------------------|---------------------:|
| [Accessibility](https://accessibility.mit.edu/) | © Russ Tedrake, 2026 |
