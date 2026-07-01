---
# layout: index
layout: landing
description: Tutorial on model predictive control for mechatronic systems, from fundamentals to deployment using the Impact toolchain
truck_trailer_video_id: SBNwXVDfLDg
xplanar_video_id: zhZ2Ko5VxUk
# drone_racing_video_id: bEeHD49rx20
bin_picking_video_id: iULN3skmdjs
overhead_crane_video_id: CcW1ONuOKA4
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

In this tutorial, participants will engage in hands-on exploration of MPC applied to mechatronic systems using Impact [1][2][8]. Impact is a flexible toolchain for the specification, prototyping, and deployment of optimal control problems (OCP) and model predictive control (MPC) strategies, with automatic generation of deployable artifacts. Impact is built on top of the Rockit toolkit for OCPs [3] and the highly popular CasADi symbolic framework for numerical optimization [4].

The key contribution of the toolchain, and this workshop, is to reduce the engineering complexity of MPC implementations by providing:

1. an intuitive symbolic tool with abstraction of technical details that are cumbersome to implement, and
2. flexible integration with state-of-the-art numerical optimization solvers, such as acados [3], Fatrop [4], and GRAMPC [5], 
    for rapid prototyping and high-performance embedded deployment.

Impact is written in Python and offers bindings for MATLAB. The generated artifacts can be executed from C/C++, Python, MATLAB, Simulink, and ROS 2 environments, and easily deployed in simulation and on hardware. Workshop exercises will be in Python. Attendees will gain practical experience and can adopt the presented open-source software frameworks in their research and applications.

The following videos show previous works developed by the MECO Research Team using the software tools that will be used in this workshop:

{% include youtubePlayer.html id=page.truck_trailer_video_id %}

{% include youtubePlayer.html id=page.xplanar_video_id %}

{% include youtubePlayer.html id=overhead_crane_video_id %}


### Registration

Registration for this workshop is managed through the CCTA conference registration system.

***

### Important dates

- Workshop date: August 11, 2026

***

### Organizers

This workshop is organized (and its content has been created) by: 

[Alvaro Javier Flórez Martínez](https://www.mech.kuleuven.be/en/pma/research/meco/people/00142153)  
Doctoral researcher

[Jan Swevers](https://www.mech.kuleuven.be/en/pma/research/meco/people/00015548)  
Full professor

[Joris Gillis](https://www.mech.kuleuven.be/en/pma/research/meco/people/00052373)  
Research scientist

[Louis Callens](https://www.mech.kuleuven.be/en/pma/research/meco/people/00143705)  
Doctoral researcher

[Mathias Bos](https://www.mech.kuleuven.be/en/pma/research/meco/people/00110026)  
Postdoctoral researcher

[Wilm Decré](https://www.mech.kuleuven.be/en/pma/research/meco/people/00052672)  
Associate professor

***

<!-- ### Contact

For any questions, please feel free to contact the organizers at:

    wilm.decre <at> kuleuven.be

***  -->
### Relevant repositories