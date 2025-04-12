# Estimation aware Trajectory optimization
An implementation of Estimation-aware Trajectory planning algorithm where the output uncertianties are state dependent. We show that to improve estimation along a trajectory while performing some taks, alternative paths are needed. We present the framework in our paper [A. Deole, M. Mesbahi](https://arxiv.org/abs/2501.09192). The estimation aware setup is applicable in general robotics applications if sensor performances have variability across state space. We present a scenario for Ego-Target Rendezvous problem where output uncertainty depends on the illumination. We present the following example:

A Demo of trajectory optimization showing Ego camera view. Here the Ego spacecraft starts tracking the Target and has to reach within 10m of the Target. The 
trajectory we design achieves this while getting to a relative position that imporves estimation. Note that we start at a position with bad sun angle and end up with best visibility under the given perception map.

<img src="https://github.com/user-attachments/assets/0fcdbcd3-5082-466f-9987-a8ec4bc3cecc" width="260" height="260">
<img src="https://github.com/user-attachments/assets/f8daa485-a0e1-4cb3-a686-a0ce0b6c52bb" width="260" height="260">

The ego trajectory is shown in blue. The red line shows direction of sun-rays. The trajectory end point is where its relative position wrt Target(red dot) and sun-rays are parallel. The video shows trajectory generated as being simulated our platform. 

The Target here is a uncooperative satellite with unknown rotation in Low-Earth orbit. The Ego satellite is parked in a relative elliptical orbit around the Target as a starting point.


<!--https://github.com/user-attachments/assets/8add4fe1-832a-43e6-a161-e226e89abac3-->


https://github.com/user-attachments/assets/86be17af-314f-440d-815d-a8d90f6a13e8



We need the Ego spacecraft to be parked at a fixed distance from the Target while saving fuel. The estimation aware trajectory desined here find a trajectory such that relative pose estimation between Target and Ego is optimized as well. We oserve that the final state achieved is such that the sun is directly behind the Ego spacecraft givng the best veiwing conditions.


The algorithm can be adapted for other robotics applications if there exists a convex enveloping function as defined [here](https://arxiv.org/abs/2501.09192). If the output map is more complex then a globally convex function, then only local convexity is required as SCVx algorithm can be used to limit exploration within a trust region. 
