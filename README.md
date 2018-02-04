# RViz textured sphere plugin
This RViz plugin takes two panospheric images and blends them on a skysphere. The sphere is created directly in OGRE and blending is GL-accelerated. Credit goes to Felipe Bacim whos rviz\_textured\_quads package helped me to get started.

## Usage

 Make sure you are publishing a sensor\_msgs/Image topic for each hemisphere.
 Load the rviz\_textured\_sphere plugin and select appropriate image topics.

`roslaunch rviz_textured_sphere demo.launch`

## What can be configured?
* image topics
* image transport methods (compression) 
* field of view of the images
* region size that will be blended
