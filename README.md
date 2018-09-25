README :
========
In this work, advanced model-based control scheme is implemented for youBot manipulator in order to achieve precise 
trajectory tracking control, and presents findings on dynamic model parameters estimation, friction modeling and 
compensation.

<img src="https://github.com/jeyaprakashrajagopal/ACCURATE-TRAJECTORY-TRACKING-ON-THE-KUKA-YOUBOT-MANIPULATOR/blob/master/tex/MT_presentation/images/softwaredesign.png" width="500">

Directories:
============
MT-Proposal : Contains thesis proposal
code        : Master thesis source
demo        : Demos include gravity compensation and trajectory tracking
experiments : Plotting details and experiment data
tex         : Tex directories of master thesis report and presentation

Dependencies:
============
This package is dependent on the following libraries

1. Orocos KDL library - https://github.com/orocos/orocos_kinematics_dynamics/tree/master/orocos_kdl
2. Simbody - https://github.com/simbody/simbody
3. youBot driver(cmake version) - https://github.com/youbot/youbot_driver
