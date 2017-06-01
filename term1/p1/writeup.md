# **Finding Lane Lines on the Road** 

**Finding Lane Lines on the Road**

![first video](solidWhiteRight_nosmooting.mp4)

### Reflection

### 1. My pipeline

My pipeline consisted of 5 steps. 
1) Converting to black and white
2) Blurring image with GaussianBlur (kernel size = 7)
3) Finding edges with Canny
4) Masking trapezoid area for "cleanup"
5) Finding lines with HoughLinesP function

You can see it using   
![jpg pipeline](single image pipeline.ipynb)

### 2. Calculations

1) Slopes - left / right line
At this point I had detected both lines (left and right) in one list/array.
I calculted slope for each line so i could distinguish right from left.
I removed some of the noise at this point by setting some limits for slopes.

2) Sorting 
To avoid any issues with drawing lines, i sorted lines by starting x

3) Averaging line


### 2. Potential shortcomings 


One potential shortcoming would be what would happen when ... 

Another shortcoming could be ...


### 3. Possible improvements to your pipeline

A possible improvement would be to ...

Another potential improvement could be to ...
