# FPV_RandomTeam

# CV Frame_Center_Detection

## Steps followed :

<br>

### 1. Convert image into HSV and use S channel(has better contrast) for further use.<br>

### 2. Use Gaussian Blur for noise removal and use Canny for edge detetction.<br>

### 3. If somehow Canny is unable to detect edges convert the image into binary image by using thresholding and then use canny.<br>

### 4.Find contours and reject all the unecessary contours using Convex Hull.<br>

```
Using Convex Hull for contour rejection(filter)


a. The convex hull produces contours as of a rubber band has been stretched over it.

b. If the area of the convex hull differs from the original area too greatly, the contour involves the support as well.

c.If the ratio of the hull area and the original area is greater then a fixed threshold, then we can reject that contour.

d.Out of the valid contours the one with the largest area is selected . The reasoning is that , due to noise, the camera may end up detecting non existent tiny contours , that must be rejected.

```

<br>

### 5.Use HoughLinesProbabilistic function on the final contour selected to apprximate several lines and average out the points on the lines to find center.<br>

<br>
<br>
<br>

## This is the basic logic behind center detection used in our code.
