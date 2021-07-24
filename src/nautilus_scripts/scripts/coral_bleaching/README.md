# Coral Bleaching Runner

## How to run
1. Launch the sim with the subway car prop with `roslaunch nautilus_worlds nautilus_sim.launch prop:=colony_dead_bleach`
2. Run the driver for the subway car photomosaic task with `rosrun nautilus_scripts coral_bleaching_runner.py`

## Description
This node takes in an image from the camera feed, and a given image, and outputs the annotated version of the camera feed
to show the differences between the two.

This script does not have any autonomous movement. The driver must position the ROV so that it views the coral reef, and
signal to take the picture manually. The original input image (not the image from the camera feed) must be at the specified
path in coral_bleaching_runner.py (original_image_path).

### Inputs
- `cam` - Camera feed (front facing)
    - captures the image to use as the current frame
- `button` - Button input topic to capture the feed
- `original_image_path` - Path to the original image to be used for comparison

### Outputs
- `old_coral_output_path` - path for old coral image with colored bounding boxes around the changes
- `new_coral_ouptut_path` - path for new coral image with colored bounding boxes around the changes
