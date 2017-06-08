[image1]: ./threshold_image.png
[image2]: ./coordinate_transform.png
[image3]: ./process_image.png
[image4]: ./roversim_screenshot.png
[image5]: ./rover_settings.png

# Search and Sample Return Project

## Rover Analysis Jupyter Notebook

I've implemented the following in the notebook:

### Running notebook and simulator, loading data
* Recorded data from simulator and stored in on disk.
* Loaded data in the notebook from the recorded data folder

### Thresholding for rover image (to discriminate sample, blocked and unblocked areas)
* Implemented thresholding in the color_thresh function such that the image returned has different integer values when a pixel/coordinate in the image is navigable/obstacle/gold sample.
* An example thresholded image is given below. It shows orange areas are blocked, white area is the sample, and black area is navigable.

![image1]

### Coordinate transform
* Applied coordinate transformations as done in the exercises
* Sample image from notebook

![image2]

### Process image
* Changed the process image function to load image data from each row of recorded data
* Process each image and create a thresholded image
* Transform coordinates and update the world map such that gold sample, obstacle and navigable areas are appropriately updated in the corresponding r/g/b channels.
* Sample output from process_image

![image3]

### Create Video
* Uploaded video to youtube [https://www.youtube.com/watc   h?v=8iebrRtWKGc]

## Rover navigation in autonomous mode

### My rover settings

![image5]

### Perception code (perception.py)
* Added process_image code and coordinate transformation code in perception.py
* Made sure I only update the world map when the roll and pitch are close to 0 or 360.
* Stored nav angles/dists and gold angles/dists for use in the decision making.

### Rover decision (decision.py)
* I have added the following checks to the decision tree
* Check for 'Stuck'
1. An ability to check if the rover is stuck.
2. If it is stuck, then stop, turn and get away from obstacle
* Stay close to left wall
1. Always calculate max navigable distance in the range [35, 40] degrees of the rover.
2. If the distance is less than safe distance, turn right
3. If the distance is optimal stay sort of straight.
4. If too far try to find left wall.
* Pickup gold
1. At any time gold is seen, rover locks to it.
2. reduces speed and starts approaching it.
3. Stops when it is near sample, and wait till pickup.
4. When done, resume.

Screenshot of rover run below:
![image4]