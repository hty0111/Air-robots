Use the script "test_trajectory.m" as the main entry point.

readonly folder: supposed to be "read only"
    quadModel_readonly.m: parameters of a 500g quadrotor
    quadEOM_readonly.m: dynamics model of quadrotor.
    run_trajectory_readonly: solve the equation of motion, receive desired trajectory, run your controller, iteratively. visualization is also included.

utils: useful functions.

test_trajectory.m: main entry.

-----------------------------------------------------------------------------------------------------------
controller.m: what you need to work with. Calculate force and moment given current and desired state vector of quadrotor.

*_trajectory.m: what you need to work with. design the trajectory for quadrotor given the path. Calculate desired state given current state vector and current time. 
Note: In case of hovering, you only need to read and check "hover_trajectory.m". If you are interested in generating different trajectories, you can refer to "diamond trajectory.m" and "circle_trajectory.m". 

Contact TAs with any questions you may have.

fake_zhiwei@zju.edu.cn
hkye@zju.edu.cn

