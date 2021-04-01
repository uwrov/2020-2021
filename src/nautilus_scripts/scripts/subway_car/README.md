# Subway Car Runner

## How to run
1. Launch the sim with the subway car prop with `roslaunch nautilus_worlds nautilus_sim.launch prop:=subway_car`
2. Run the driver for the subway car photomosaic task with `rosrun nautilus_scripts subway_car_runner.py`.

## Description
This node takes in images from the camera feed on command and stitches them into a photomosaic.

This script does not have any autonomous movement. The driver must position the ROV so that it views each face of the subway car, and signal to take the picture manually.

The script expects the driver to take the pictures in this order:
```[end, side, end, side, top]```

With an end being a square face and a side being a long face. So drive the ROV around the car, then fly above it. When taking an image from the top try to get it in the same angle as you would with one of the sides.

### Inputs
- `cam1` - Camera feed 1 topic (front facing)
- `cam2` - Camera feed 2 topic (downward facing)
- `button` - Button input topic
    - Currently subscribes to a string topic, on receiving "1" takes frame from camera feed 1, on receiving "2" takes frame from camera feed 2.
    - Can change the message as you want.

### Outputs
- `output_path` - Currently this script will just save the final photomosaic to an output_path specified with a variable, feel free to change how the output is handled. 