# RViz textured sphere plugin
This RViz plugin takes two panospheric images and blends them on a skysphere. The sphere is created directly in OGRE and blending is GL-accelerated. Credit goes to Felipe Bacim whos rviz\_textured\_quads package helped me to get started.

![](media/rviz_textured_sphere_demo.gif)

## Usage

 Make sure you are publishing a sensor\_msgs/Image topic for each hemisphere.
 Load the rviz\_textured\_sphere plugin and select appropriate image topics.

`roslaunch rviz_textured_sphere demo.launch`

## What can be configured?
* image topics
* image transport methods (compression) 
* field of view of the images
* region size that will be blended

## References
More information about this package is available in the paper [Improved Situational Awareness in ROS Using Panospheric Vision and Virtual Reality](https://doi.org/10.1109/HSI.2018.8431062).
If you are using this software please add the following citation to your publication:
```
@INPROCEEDINGS{VunderSA2018, 
author={V. Vunder and R. Valner and C. McMahon and K. Kruusamäe and M. Pryor}, 
booktitle={2018 11th International Conference on Human System Interaction (HSI)}, 
title={Improved Situational Awareness in ROS Using Panospheric Vision and Virtual Reality}, 
year={2018}, 
pages={471-477}, 
keywords={Robots;Data visualization;Cameras;Headphones;Lenses;Distortion;Rendering (computer graphics);situational awareness;human-robot interaction;virtual reality;user interfaces;panospheric vision;telerobotics;ROS;RViz}, 
doi={10.1109/HSI.2018.8431062}, 
month={July},}
```
