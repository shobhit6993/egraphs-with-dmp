##Incorporating Online Obstacle Avoidance in E-Graphs
Planning with Experience-graphs (E-graphs) <cite>[1]</cite> speeds up motion planning
by leveraging previous planning episodes, while providing
provable bounds on the sub-optimality of the generated plans.
However, E-graphs assume a static environment and cannot
handle unseen or moving obstacles. To address this issue, 
an integration of potential fields based online
obstacle avoidance using DMP framework into the E-graphs
framework is tested. The proposed integration allows E-graphs to adapt
to unseen obstacles, while avoiding the need for re-planning on
account of obstacle avoidance.

####Files
- A write-up of the proposed framework is included in ``report.pdf``.
- ``egraphs`` and ``egraphs_vis`` contains the codebase for egraphs imported from the authors' repositories.


####References
[1]: Phillips, Mike, et al. "E-Graphs: Bootstrapping Planning with Experience Graphs." Robotics: Science and Systems. Vol. 5. No. 1. 2012.
