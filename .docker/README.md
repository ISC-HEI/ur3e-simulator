# ISCoin Simulator Docker Containers

## Build

> :information_source: building the image from source likely won't be needed 

From the top-level, build the container with
```bash
docker build -t iscoin-simulator -f .docker/Dockerfile .
```

## Pull

A pre-built image has been pushed to the local Docker registry on Calypso. To be able to pull from the remote registry, the following key-value pair needs to be added to the local machine's `/etc/docker/daemon.json` (at top-level):

```bash
"insecure-registries": ["192.168.88.248:5000"]
```

Docker should then be restarted with

```bash
sudo systemctl restart docker
```

To test the connection, the registry's catalog can be queried with:

```bash
curl 192.168.88.248:5000/v2/_catalog
```

Finally, the simulator container can be pulled:

```bash
docker pull 192.168.88.248:5000/iscoin-simulator
```

## Run

Depending on hardware availability, run the container with `docker compose` and either the `gpu` or `cpu` argument

```bash
docker compose run --rm --name iscoin_simulator gpu|cpu
```

The UR3e simulator can be started with

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

To quit the simulator, close the window or use `CTRL+C` in the corresponding terminal.