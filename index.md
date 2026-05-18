---
# layout: index
layout: landing
description: Tutorial on model predictive control for mechatronic systems, from fundamentals to deployment using the Impact toolchain
truck_trailer_video_id: SBNwXVDfLDg
xplanar_video_id: zhZ2Ko5VxUk
# drone_racing_video_id: bEeHD49rx20
bin_picking_video_id: iULN3skmdjs
---


[Registration](#registration){: .btn .btn-primary .d-lg-inline-block my-lg-0 .mt-1}
[Important dates](#important-dates){: .btn .btn-primary .d-lg-inline-block my-lg-0 my-2 mx-lg-2 .mt-1}
[Organizers](#organizers){: .btn .btn-primary .d-lg-inline-block my-lg-0 my-2 mx-lg-2 .mt-1}
[Contact](#contact){: .btn .btn-primary .d-lg-inline-block my-lg-0 .mt-1} 
 
[Relevant repositories](#relevant-repositories){: .btn .btn-primary .d-lg-inline-block my-lg-0 .mt-1}
[Program](#program){: .btn .btn-primary .d-lg-inline-block my-lg-0 .mt-1}
[Venue](#venue){: .btn .btn-primary .d-lg-inline-block my-lg-0 my-2 mx-lg-2 .mt-1}

***

### Overview

Model Predictive Control is a well-established technique for controlling (possibly) nonlinear systems that has predictive ability and can cope with constraints. It has a strong track record in chemical engineering. More recent advances in problem formulations and high-performance solvers have expanded MPC’s applicability to domains such as power systems, mechatronics, and robotics, with complex dynamics and high sampling rates.

In this tutorial, participants will engage in hands-on exploration of MPC applied to mechatronic systems using Impact [1][2]. Impact is a flexible toolchain for the specification, prototyping, and deployment of optimal control problems (OCP) and model predictive control (MPC) strategies, with automatic generation of deployable artifacts. Impact is built on top of the Rockit toolkit for OCPs [3] and the highly popular CasADi symbolic framework for numerical optimization [4].

The key contribution of the toolchain, and this workshop, is to reduce the engineering complexity of MPC implementations by providing:

1. an intuitive symbolic tool with abstraction of technical details that are cumbersome to implement, and
2. flexible integration with state-of-the-art numerical optimization solvers, such as acados [3], Fatrop [4], and GRAMPC [5], 
    for rapid prototyping and high-performance embedded deployment.

Impact is written in Python and offers bindings for MATLAB. The generated artifacts can be executed from C/C++, Python, MATLAB, Simulink, and ROS 2 environments, and easily deployed in simulation and on hardware. Workshop exercises will be in Python. Attendees will gain practical experience and can adopt the presented open-source software frameworks in their research and applications.

[1] A. Florez, A. Astudillo, W. Decré, J. Swevers, and J. Gillis, "IMPACT: A Toolchain for Nonlinear Model Predictive Control Specification, Prototyping, and Deployment", IFAC-PapersOnLine, vol. 56, no. 2, pp. 3164–3169, 2023, doi: https://doi.org/10.1016/j.ifacol.2023.10.1451.

[2] A. Astudillo, A. Florez, W. Decré, and J. Swevers, “Rapid Deployment of Model Predictive Control for Robotic Systems: From IMPACT to ROS 2 Through Code Generation”, in Proceedings of the 2024 IEEE 18th International Conference on Advanced Motion Control (AMC), 2024, doi: https://doi.org/10.1109/amc58169.2024.10505632.

[3] Gillis, J., Vandewal, B., Pipeleers, G., Swevers, J., “Effortless modeling of optimal control problems with rockit”, 39th Benelux Meeting on Systems and Control 2020, Elspeet, The Netherlands.

[4] Andersson, J.A.E., Gillis, J., Horn, G. et al. CasADi: a software framework for nonlinear optimization and optimal control. Math. Prog. Comp. 11, 1–36 (2019), doi: https://doi.org/10.1007/s12532-018-0139-4.

[5] Verschueren, R., Frison, G., Kouzoupis, D. et al. acados—a modular open-source framework for fast embedded optimal control. Math. Prog. Comp. 14, 147–183 (2022), doi:https://doi.org/10.1007/s12532-021-00208-8.

[6] L. Vanroye, A. Sathya, J. De Schutter and W. Decré, “FATROP: A Fast Constrained Optimal Control Problem Solver for Robot Trajectory Optimization and Control,” 2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Detroit, MI, USA, 2023, pp. 10036-10043, doi: https://doi.org/10.1109/IROS55552.2023.10342336.

[7] Englert, T., Völz, A., Mesmer, F. et al. A software framework for embedded nonlinear model predictive control using a gradient-based augmented Lagrangian approach (GRAMPC). Optim Eng 20, 769–809 (2019), doi:  https://doi.org/10.1007/s11081-018-9417-2.


The following videos show previous works developed by the MECO Research Team using the software tools that will be used in this workshop:

{% include youtubePlayer.html id=page.truck_trailer_video_id %}

{% include youtubePlayer.html id=page.xplanar_video_id %}

{% include youtubePlayer.html id=page.bin_picking_video_id %}

### Registration

Registration for this workshop is managed through the CCTA conference registration system.

***

### Important dates

- Workshop date: August 11, 2026

***

### Organizers

This workshop is organized (and its content has been created) by: 

[Alvaro Javier Florez Martínez](https://www.mech.kuleuven.be/en/pma/research/meco/people/00142153)  
Doctoral researcher

[Joris Gillis](https://www.mech.kuleuven.be/en/pma/research/meco/people/00052373)  
Research scientist

[Mathias Bos](https://www.mech.kuleuven.be/en/pma/research/meco/people/00110026)  
Postdoctoral researcher

[Wilm Decré](https://www.mech.kuleuven.be/en/pma/research/meco/people/00052672)  
Associate professor

[Louis Callens](https://www.mech.kuleuven.be/en/pma/research/meco/people/00143705)  
Doctoral researcher

[Jan Swevers](https://www.mech.kuleuven.be/en/pma/research/meco/people/00015548)  
Full professor

***

### Contact

For any questions, please feel free to contact the organizers at:

    wilm.decre <at> kuleuven.be

*** 
### Relevant repositories