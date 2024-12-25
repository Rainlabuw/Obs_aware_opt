# Estimation aware Trajectory obtimization
A Demo of trajectory optimization showing Ego camera view. Here the Ego spacecraft starts tracking the Target and has to reach within 10m of the Target. The 
trajectory we design achieves this while getting to a relative position that imporves estimation. Note that we start at a position with bad sun angle and end up with best visibility under the given perception map.


![MPC_trjf](https://github.com/Rainlabuw/Observability_MPC/assets/33707695/66324750-156f-41fb-9616-ea220bd9b787)

The ego trajectory is shown in blue. The red line shows direction of sun-rays. The trajectory end point is where its relative position wrt Target(red dot) and sun-rays are parallel. The video shows trajectory generated as being simulated our platform. 

The Target here is a uncooperative satellite with unknown rotation in Low-Earth orbit. The Ego satellite is parked in a relative elliptical orbit around the Target as a starting point.

https://github.com/Rainlabuw/Observability_MPC/assets/33707695/723123dc-a502-4c43-987c-07bd7641554d

We need the Ego spacecraft to be parked at a fixed distance from the Target while saving fuel. The estimation aware trajectory desined here find a trajectory such that relative pose estimation between Target and Ego is optimized as well. We oserve that the final state achieved is such that the sun is directly behind the Ego spacecraft givng the best veiwing conditions.
