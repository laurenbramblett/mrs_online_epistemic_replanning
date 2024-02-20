## Robust Online Epistemic Replanning of Multi-Agent Missions
This code was tested in MATLAB 2023. The following toolboxes are required or equivalent functions missing:
- Base: `MATLAB`; `Statistics and Machine Learning Toolbox`
- If Running Robots: `ROS Toolbox`

#### Running main script with example centralized plan output
Please follow the steps below to run an example simulation of the algorithm submitted to IROS

- Open `main_run.m` in MATLAB
- Modify any parameters in lines 6-47 to include the robot parameters and the failure conditions
- Run the code based on the centralized planner example output from `examples/allocation3bots.txt` and `examples/instance3bots.txt` which define the task assignment and task parameters respectively.

The output will be a video simulation and populated data table incorporating user-defined failures during the mission. The input is an example output from the centralized planning executable which plans optimized rendezvous points during a mTSP operation. 

#### Running main script and running centralized plan (Requires Windows OS)
Please follow the main steps below for including the centralized planner in your simulation run
- Open `centralized_plan\Release\instance_3.txt`
- Replace the header above the first `[-1,-1]` row with the starting poses of the robots (the number of rows with starting poses equals the number of robots)
- Replace the next rows after the first `[-1,-1]` row with the task locations
- The last row will be the shared depot location for all robots
- Open `main_run_withGA.m` in MATLAB
- Set `run_exe = true` and modify any user-defined parameters in lines 2 - 67

A user may also wish to produce the crazyflie experiment results by using the utilities provided and setting `real_robots = true`; however, more setup is required to get the crazyflies operational before running the code which can be found here:
https://crazyswarm.readthedocs.io/en/latest/installation.html

If using this repository, please cite our work:
```
@article{bramblett2023epistemic,
  title={Epistemic planning for multi-robot systems in communication-restricted environments},
  author={Bramblett, Lauren and Bezzo, Nicola},
  journal={Frontiers in Robotics and AI},
  volume={10},
  pages={1149439},
  year={2023},
  publisher={Frontiers}
}
@article{bramblett2023epistemic,
  title={Epistemic Prediction and Planning with Implicit Coordination for Multi-Robot Teams in Communication Restricted Environments},
  author={Bramblett, Lauren and Gao, Shijie and Bezzo, Nicola},
  journal={arXiv preprint arXiv:2302.10393},
  year={2023}
}
```
