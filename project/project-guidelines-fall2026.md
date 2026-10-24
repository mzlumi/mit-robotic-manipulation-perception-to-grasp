<!-- Source: https://manipulation.csail.mit.edu/Fall2026/project.html (retrieved 2026-10-27). Automatic conversion with pandoc from the saved HTML next to this file. -->

Toggle navigation Menu

[MIT 6.4210/6.4212 - Robotic Manipulation](index.html)

- [Description](index.html#description)
- [Information](index.html#information)
- [Schedule](schedule.html)
- [Project](project.html)
- [FAQ](index.html#faq)
- [People](index.html#people)

## Final Project

The final project will be your opportunity to take what you learned in the class and apply it to a project of your choice.

Here are some guidelines:

- The project topic is flexible but it should be directly relevant to robot manipulation and build on material you have learned in this course.
- **All projects should have teams of 3 people**. The team members should be from the same subject (6.4210 or 6.4212).
- The project is only over 4-5 weeks, so scope it carefully. Think of it as about 40 x 3 = 120 person-hours of work.
- Developing novel algorithms/methods is great, but re-implementing published work and/or applying a published method to a manipulation system is also great. You will be evaluated based on whether the project forced you to understand new concepts, not based on the novelty of the approach.
- We will expect every project to compare two or more approaches to solving the stated problem. Every robotics paper includes "baselines" that establish the level of performance from an existing approach. For example, if you propose to learn a policy to do a task, how would a "classical" method work?
- Using Drake is not a requirement, but the staff can help you more if you choose this route.
- It is fine if your project is related to research work that one or more group members are currently pursuing. But the project should go beyond existing work and must still make meaningful connection to the material of this class.
- We encourage you to build on code and tools of the community, but remember that you must clearly distinguish your contribution and acknowledge the work of others to avoid plagiarism. If you do improve an open-source tool or example, consider contributing back!
- You are free to use coding agents, but you are responsible for owning and understanding all the *robotics-level* design choices. So, you don't need to know/remember whether something was stored in a hash-table or which Drake API calls were made; but you do need to know and be able to justify your choice of robot algorithm (e.g., RRT vs continuous optimization; or antipodal vs force-closure as a grasp criterion).
- If you select a project with a machine-learning component, it will be evaluated on the quality of the ideas and project framing, not on how much GPU time you burn or how excellent the actual performance is on a hard problem. Focus on ideas, and demonstrations in simple problems that can be addressed without huge amounts of compute. We will post some additional pointers on computational resources, but *don't feel you need to use a lot of compute!*
- We've included a few [project ideas](#ideas) below. You can find some great final projects from previous semesters [here](https://manipulation.csail.mit.edu/misc.html#projects). Note, though, that in previous semesters we did not have the requirement that there be three members in a group and did not require at least two approaches to a problem.

The course staff will provide you with feedback on the scope on direction of your proposal shortly after we receive it.

### Project Proposal

The project proposal is intended as a forcing function for you to crystallize a project idea. Moreover, it gives us a chance to offer you a feedback and make sure that your plan is feasible in the time allotted to the project. We'll split it in two phases: (1) pre-proposal - a chance for you to list one or more rough ideas for your project and (2) final proposal - a more in-depth description of the proposed project. The pre-proposal should be ~500-750 words, and the final version ~1000 words. **Note that CI-M students will receive a more detailed rubric in the CI-M recitations.** The proposal should detail your idea for the project, and:

- Define exactly what the project deliverable is.
- Briefly describe why the project is interesting.
- List the topics that we have studied (or that we will study) in class that are covered by your project. For example: kinematics, deep perception, force control, motion planning, behavior cloning, etc.
- Discuss any related prior work you have found that is relevant. **If this project is related to your research or a project you are doing concurrently for another class**, tell us about that now (and read the guidelines for the final report below).
- Define specific goals that you expect to have accomplished before each of the progress updates.
- Your proposal should have a few sentences describing how the work will be divided across the team. It’s OK if plans change, but we want to make sure that the initial roles are clear, and that the project makes sense for multiple people. And, remember that distributing primary responsibility does not mean that you can ignore all the details of that part. We expect that you will understand all parts of the project and be able to answer technical questions about it.
  It does not have to be extremely polished, but it does need to provide us with enough information to understand what you are hoping to do. Otherwise, we won't be able to help you!

Here are a couple of example project proposals from previous years:

- [Rock Skipping Robot](./example_proposals/rock_skipping_proposal.pdf) by Michael Burgess & Nicholas Ramirez. (Graduate)
- [Throwing and Catching Bot](./example_proposals/throwing_and_catching_proposal.pdf) by Quincy Johnson, Hanqi Su, and Arif Dayi. (Undergraduate)

### Progress Updates

We will request short (~1 paragraph) project updates in order to keep us up to date with how you have progressed. 6.4210 students will have scheduled interactions with the CI-M staff. All students will also meet with the technical staff to discuss their projects/progress. These check-ins will be roughly weekly and scheduled with the TA assigned to each group. The meetings will start after the pre-proposal is due (and after Quiz 1). You should have at least one meeting before your final proposal is due.

### Final Video

The project videos must be around 5 minutes for three-person projects. For group videos, please try to share the time evenly between the various members of the group. This year we will require you to upload videos on YouTube; more details will follow on Piazza.

### Final Report

For your final report, you should use the [IEEE Template for conference proceedings](https://www.ieee.org/conferences/publishing/templates.html) (probably the LaTeX one, unless you really enjoy using MS Word). Write a summary of what you accomplished during your project. Write it, as much as possible, like a conference paper. You should include:

- An abstract.
- An introduction of your project and why you think it is interesting.
- A related work / literature review section. Focus on the most closely related papers.
- Your technical approach / methods
- Your results, including a comparison between approaches (partial or work-in-progress is expected, and completely fine!)
- A discussion of your results and potential next steps
- **A description of the contributions made by each team member**.
- For projects related to a team-member's research or a project in another class, **please clearly denote what parts of the project overlap (or not) with the other efforts**.
- Describe how you used AI in the project. Pay careful attention to the AI policy posted on the web page.

**CI-M students will receive a more detailed rubric from CI-M recitations** For projects that do overlap with your research, please make connections to the topics in the class. A report that might have been written if you had not taken the class does not constitute a successful project. The goal of the project is to demonstrate some level of mastery of the material from class.

The report is worth a very large portion of your overall grade, so be as thorough as possible in demonstrating mastery of the course material. The report should be around 4500 words for teams of three, not counting figures and references.

### Final Report: In class summaries

During the last lecture period, we will ask each team member to answer some targeted questions about their team's project. So, make sure that you understand the whole project well; do not simply divide it into parts and ignore the other parts of the project. Your answers to these questions will be graded as part of your final project report.

### Project Ideas

Here are a set of ideas we believe might be worth investigating, arising from the class topics, but do not feel the need to grab one of these! The papers linked are simple place-holders, you should look for other related work. In most cases, re-implementing one of these papers is too big a project. But taking the ideas from one of these papers and applying them in a (much) simpler context could be good.

#### Inverse kinematics and trajectories

Inverse kinematics via learning

- Zhang & Jiao, "[IKDiffuser: A Generative Inverse Kinematics Solver for Multi-arm Robots via Diffusion Model](https://arxiv.org/abs/2506.13087)," 2025.
- Ames et al., "[IKFlow: Generating Diverse Inverse Kinematics Solutions](https://arxiv.org/abs/2111.08933)," RA-L 2022.

Optimization-based trajectory optimization

- Yoon et al., "[Learning-based Initialization of Trajectory Optimization for Path-following Problems of Redundant Manipulators](https://arxiv.org/pdf/2602.03418)."

#### Motion planning

VAMP vs. neural motion planning

- Thomason et al., "[Motions in Microseconds via Vectorized Sampling-Based Planning](https://github.com/KavrakiLab/vamp)" (VAMP), ICRA 2024.
- Dalal et al., "[Neural MP: A Generalist Neural Motion Planner](https://arxiv.org/abs/2409.05864)," IROS 2025.
- Fishman et al., "[Motion Policy Networks](https://arxiv.org/abs/2210.12209)," CoRL 2022.
- Sundaralingam et al., "[cuRobo: Parallelized Collision-Free Minimum-Jerk Robot Motion Generation](https://arxiv.org/abs/2310.17274)," extended technical report for ICRA 2023.
- Soleymanzadeh et al., "[Toward Generalist Neural Motion Planners for Robotic Manipulators: Challenges and Opportunities](https://arxiv.org/abs/2603.24318)," 2026.

Non-holonomic motion planning

- Li et al., "[MPC-MPNet: Model-Predictive Motion Planning Networks for Fast, Near-Optimal Planning under Kinodynamic Constraints](https://arxiv.org/abs/2101.06798)," RA-L 2021.
- Johnson et al., "[Dynamically Constrained Motion Planning Networks for Non-Holonomic Robots](https://arxiv.org/abs/2008.05112)," IROS 2020.
- Carvalho et al., "[Motion Planning Diffusion: Learning and Planning of Robot Motions with Diffusion Models](https://arxiv.org/abs/2308.01557)," IROS 2023.

Fast asymptotically optimal sample-based planning

- Wilson et al., "[Nearest-Neighbourless Asymptotically Optimal Motion Planning with Fully Connected Informed Trees (FCIT\*)](https://arxiv.org/abs/2411.17902)," ICRA 2025.
- Wilson et al., "[AORRTC: Almost-Surely Asymptotically Optimal Planning with RRT-Connect](https://arxiv.org/abs/2505.10542)," RA-L 2025 (implemented in the VAMP repo).
- Strub & Gammell, "[Adaptively Informed Trees (AIT\*) and Effort Informed Trees (EIT\*): Asymmetric bidirectional sampling-based path planning](https://arxiv.org/abs/2111.01877)," IJRR 2022.

#### Grasping

Comparing learned models with heuristic methods

- Fang et al., "[AnyGrasp: Robust and Efficient Grasp Perception in Spatial and Temporal Domains](https://arxiv.org/abs/2212.08333)," T-RO 2023.
- Sundermeyer et al., "[Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes](https://arxiv.org/abs/2103.14127)," ICRA 2021 ([code](https://github.com/NVlabs/contact_graspnet)).
- Fang et al., "[GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping](https://openaccess.thecvf.com/content_CVPR_2020/html/Fang_GraspNet-1Billion_A_Large-Scale_Benchmark_for_General_Object_Grasping_CVPR_2020_paper.html)," CVPR 2020 (IJRR 2023 extended version).

Grasping from single RGB images

- Zhai et al., "[MonoGraspNet: 6-DoF Grasping with a Single RGB Image](https://arxiv.org/abs/2209.13036)," ICRA 2023.
- Guo et al., "[Monocular One-Shot Metric-Depth Alignment for RGB-Based Robot Grasping](https://arxiv.org/abs/2506.17110)," 2025.
- Liu et al., "[RGBGrasp: Image-Based Object Grasping by Capturing Multiple Views during Robot Arm Movement with Neural Radiance Fields](https://arxiv.org/abs/2311.16592)," RA-L 2024.

#### Control

Capturing inertial/friction parameters from interaction

- Memmel et al., "[ASID: Active Exploration for System Identification in Robotic Manipulation](https://arxiv.org/abs/2404.12308)," ICLR 2024.
- Jatavallabhula et al., "[gradSim: Differentiable Simulation for System Identification and Visuomotor Control](https://arxiv.org/abs/2104.02646)," ICLR 2021.
- He et al., "[RigPI: Dynamic Parameter Identification of Rigid Body via VLM-Seeded Differentiable Simulation](https://arxiv.org/abs/2606.25212)," 2026.

Non-prehensile manipulation

- Pezzato et al., "[Sampling-Based Model Predictive Control Leveraging Parallelizable Physics Simulations](https://arxiv.org/abs/2307.09105)," RA-L 2025.
- Howell et al., "[Predictive Sampling: Real-time Behaviour Synthesis with MuJoCo](https://arxiv.org/abs/2212.00541)," 2022 (MuJoCo MPC [code](https://github.com/google-deepmind/mujoco_mpc)).
- Raicevic et al., "[Object-Informed Model Predictive Path Integral Control for Non-Prehensile Robot Manipulation](https://arxiv.org/abs/2605.30778)," 2026.

Compliant strategies for assembly

- Tang et al., "[IndustReal: Transferring Contact-Rich Assembly Tasks from Simulation to Reality](https://arxiv.org/abs/2305.17110)," RSS 2023.
- Narang et al., "[Factory: Fast Contact for Robotic Assembly](https://arxiv.org/abs/2205.03532)," RSS 2022.
- Luo et al., "[SERL: A Software Suite for Sample-Efficient Robotic Reinforcement Learning](https://arxiv.org/abs/2401.16013)," ICRA 2024 ([project site](https://serl-robot.github.io/)).
- Ankile et al., "[From Imitation to Refinement: Residual RL for Precise Assembly](https://arxiv.org/abs/2407.16677)," ICRA 2025.

#### Task and motion planning (TAMP)

Tool use

- Trupin et al., "[Physics-Conditioned Grasping for Stable Tool Use](https://arxiv.org/abs/2505.01399)" (iTUP), 2025; the earlier [v1](https://arxiv.org/abs/2505.01399v1) is framed as "Dynamic Robot Tool Use with Vision Language Models."
- Xu et al., "[Creative Robot Tool Use with Large Language Models](https://arxiv.org/abs/2310.13065)" (RoboTool), 2023.
- Gao et al., "[VLMgineer: Vision Language Models as Robotic Toolsmiths](https://arxiv.org/abs/2507.12644)," 2025.

Balancing unusual objects

- Lee et al., "[Beyond Pick-and-Place: Tackling Robotic Stacking of Diverse Shapes](https://arxiv.org/abs/2110.06192)," CoRL 2021.
- Goldberg et al., "[Blox-Net: Generative Design-for-Robot-Assembly Using VLM Supervision, Physics Simulation, and a Robot with Reset](https://arxiv.org/abs/2409.17126)," ICRA 2025.

Large spatial scale: tidying house

- Wu et al., "[TidyBot: Personalized Robot Assistance with Large Language Models](https://arxiv.org/abs/2305.05658)," Autonomous Robots 2023.
- Liu et al., "[OK-Robot: What Really Matters in Integrating Open-Knowledge Models for Robotics](https://arxiv.org/abs/2401.12202)," RSS 2024.
- Yenamandra et al., "[HomeRobot: Open-Vocabulary Mobile Manipulation](https://arxiv.org/abs/2306.11565)," CoRL 2023.

Exploring the limits of MLLMs in these problems (what tools do they need, can they use them)

- Curtis et al., "[Trust the PRoC3S: Solving Long-Horizon Robotics Problems with LLMs and Constraint Satisfaction](https://arxiv.org/abs/2406.05572)," CoRL 2024.
- Yang et al., "[Guiding Long-Horizon Task and Motion Planning with Vision Language Models](https://arxiv.org/abs/2410.02193)," 2024.
- Yang et al., "[EmbodiedBench: Comprehensive Benchmarking Multi-modal Large Language Models for Vision-Driven Embodied Agents](https://arxiv.org/abs/2502.09560)," ICML 2025.

#### Learning

Behavior cloning trained with planner outputs

- Fishman et al., "[Motion Policy Networks](https://arxiv.org/abs/2210.12209)," CoRL 2022.
- Dalal et al., "[Neural MP: A Generalist Neural Motion Planner](https://arxiv.org/abs/2409.05864)," IROS 2025.
- Dalal et al., "[Imitating Task and Motion Planning with Visuomotor Transformers](https://arxiv.org/abs/2305.16309)" (OPTIMUS), CoRL 2023.

RL initialized with behavior cloning data

- Ankile et al., "[From Imitation to Refinement: Residual RL for Precise Assembly](https://arxiv.org/abs/2407.16677)," ICRA 2025.
- Ren et al., "[Diffusion Policy Policy Optimization](https://arxiv.org/abs/2409.00588)" (DPPO), ICLR 2025.
- Hu et al., "[Imitation Bootstrapped Reinforcement Learning](https://arxiv.org/abs/2311.02198)" (IBRL), RSS 2024.

The effects of the low-level controller on behavior cloning

- Bronars et al., "[Tune to Learn: How Controller Gains Shape Robot Policy Learning](https://arxiv.org/abs/2604.02523)," 2026.
- Aljalbout et al., "[On the Role of the Action Space in Robot Manipulation Learning and Sim-to-Real Transfer](https://arxiv.org/abs/2312.03673)," RA-L 2024.
- Chi et al., "[Diffusion Policy: Visuomotor Policy Learning via Action Diffusion](https://arxiv.org/abs/2303.04137)," RSS 2023 (see the position vs. velocity control ablation).

Learn some motor policies and get an MLLM to use them

- Ahn et al., "[Do As I Can, Not As I Say: Grounding Language in Robotic Affordances](https://arxiv.org/abs/2204.01691)" (SayCan), CoRL 2022.
- Liang et al., "[Code as Policies: Language Model Programs for Embodied Control](https://arxiv.org/abs/2209.07753)," ICRA 2023.
- Shi et al., "[Hi Robot: Open-Ended Instruction Following with Hierarchical Vision-Language-Action Models](https://arxiv.org/abs/2502.19417)," ICML 2025.
- Belkhale et al., "[RT-H: Action Hierarchies Using Language](https://arxiv.org/abs/2403.01823)," RSS 2024.

Model learning via nearest neighbor or system identification: learning in the now

- Pari et al., "[The Surprising Effectiveness of Representation Learning for Visual Imitation](https://arxiv.org/abs/2112.01511)" (VINN), RSS 2022.
- Baumeister et al., "[Incremental Few-Shot Adaptation for Non-Prehensile Object Manipulation using Parallelizable Physics Simulators](https://arxiv.org/abs/2409.13228)," 2024.
- Memmel et al., "[ASID: Active Exploration for System Identification in Robotic Manipulation](https://arxiv.org/abs/2404.12308)," ICLR 2024.

Copyright © 2018-2026, Robot Locomotion Group @ CSAIL

Web design with Bootstrap \| Contact: manipulation-staff \[AT\] mit \[dot\] edu
