# 2020-2021

## First time Setup

**If your username is 'uwrov'**, run `setup_sim.sh`.

If you do not use the VM with username 'uwrov', change any instance of `/home/uwrov` to `/home/{your username}`, in `setup_sim.sh` ***before*** running it.

After running the script, run `catkin_make` and you should be good to go!

## Launching the Simulator

Once you have all the simulator files ready, launch a sample world using

```
roslaunch nautilus_worlds nautilus_sim.launch reset_tam:=true prop:=subway_car
```

This will launch the simulator, with the nautilus facing the subway car from task 3.4.

### Command Line Arguments

`nautilus_sim.launch` has a few optional command line arguments to fine tune your simulating experience:

- `prop_x`, `prop_y`, `prop_z`: Cartesian coordinates the prop will spawn (in meters)
    - default: (0, 0, -99.5)
- `nautilus_x`, `nautilus_y`, `nautlius_z`: Cartesian coordinates nautilus will spawn (in meters)
    - default: (-3, 0, -99.5)
- `prop`: Name of the one prop you want to spawn, props listed in the `src/models/models/` folder, here are the ones currently supported:
    - default: `none`
    - `subway_car`
    - `colony_original`
    - `colony_spec`
    - `colony_dead_bleach`
- `reset_tam`: Set to true if you need to reset thruster allocation matrix
    - **Note: Must be true the first time you run the simulator**
    - default: false

## Viewing the Simulated Camera

The cameras on nautilus publish `sensor_msgs/Image` to:
- `/nautilus/nautilus/camera1/nautilus_cam`: Front facing camera
- `/nautilus/nautilus/camera2/nautilus_cam`: Downward facing camera

## Controlling Nautilus

To control nautilus, send `geometry_msgs/Wrench` messages to `/nautilus/thruster_manager/input`.

A `Wrench` message is defined like so:
```
Vector3 force
Vector3 torque
```

`force` describes the desired force in the xyz directions, while `torque` represents rotations in rpy.

Note that nautilus can only turn along the z-axis, so only the third component of `torque` is needed.

## Launching a Control Script
All the control scripts are located in [src/nautilus_scripts/scripts]{https://github.com/uwrov/2020-2021/tree/sim/src/nautilus_scripts/scripts}. You can run them by doing a `rosrun nautilus_scripts filename.py`. Currently the working scripts are:
- `subway_car_runner.py`
