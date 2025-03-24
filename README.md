# ISCoin-simulator

|![Screenshot](./img/iscoin_sim_viz.png "Example trajectory visualization")|
|:--:|
|*Example trajectory visualization*|

## Setup

See [instructions](./.docker/README.md) to build or download the containerized `iscoin_simulator`.

## Simulate joint trajectories

Depending on hardware availability, run the container with `docker compose` and either the `gpu` or `cpu` argument

```bash
docker compose run --rm --name iscoin_simulator gpu|cpu
```

Inside the container, the UR3e simulator can be started from the terminal with

```bash
ros2 launch iscoin_simulation_gz iscoin_sim_control.launch.py
```

which will open the Gazebo Ignition display locally, on the host device. To send commands to the simulated robot, another terminal window is needed. This can be done by using a terminal multiplexer like `tmux` or by entering the running container through another terminal with

```bash
docker exec -it iscoin_simulator /bin/bash
```

As an example, run the following command from the new terminal window to simulate a demo trajectory:

```bash
ros2 run iscoin_driver demo.py
```

The trajectories to be simulated are saved as `json` in the `iscoin_driver/config` directory. Custom trajectories can be passed to the demo script in the following manner:

```bash
ros2 run iscoin_driver demo.py --ros-args -p traj:=<path-to>/custom_traj.json
```

To quit the simulator, close the window or use `CTRL+C` in the corresponding terminal.

## Inverse kinematics

The inverse kinematics library is based on [ur_ikfast](https://github.com/cambel/ur_ikfast/tree/master) which has been extended to support the `iscoin` robot (i.e., UR3e + Hand-e). The library can be invoked directly from Python code in the docker environment.

```python
from ur_ikfast import ur_kinematics

iscoin_arm = ur_kinematics.URKinematics('iscoin')

joint_angles = [-3.1, -1.6, 1.6, -1.6, -1.6, 0.]  # in radians
print("joint angles", joint_angles)

pose_quat = iscoin_arm.forward(joint_angles)
pose_matrix = iscoin_arm.forward(joint_angles, 'matrix')

print("forward() quaternion \n", pose_quat)
print("forward() matrix \n", pose_matrix)

# print("inverse() all", iscoin_arm.inverse(pose_quat, True))
print("inverse() one from quat", iscoin_arm.inverse(pose_quat, False, q_guess=joint_angles))

print("inverse() one from matrix", iscoin_arm.inverse(pose_matrix, False, q_guess=joint_angles))
```