Python implementaion with Scvx for satellite rendezvous problem. The evveloping fuction is defined as $(x - x_s)^\top Q (x - x_s)$ where $x_s$ is the state corresponding to best estimation, ie the best illumination. The enveloping function has been determined by data. 

Built with cvxpy, the package requirements are minimal and listed in `requirements.txt`.

Run `scvx_obs_paper_run.py` to generate an estimation aware trajectory for the Ego Target scenario. The optimal control problem is defined here and exploration parameter defined as `obs_lambda` can be used to tune the trajectories.

Run `scvx_obs_analysis.py` to generate comparision of different exploration parameters.

Run `scvx_obs_filter2.py` to generate comparision of filters in online trajectory following mode.

