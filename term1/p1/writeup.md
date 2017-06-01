# **Finding Lane Lines on the Road** 

**Finding Lane Lines on the Road**

Movies with working algorithms:

Yellow  (without and with temporal smooting)
https://vimeo.com/219801348  
https://vimeo.com/219801360  

White   (without and with temporal smooting)
https://vimeo.com/219801499  
https://vimeo.com/219801342  

### Reflection

### 1. My pipeline

My pipeline consisted of 5 steps. 
1) Converting to black and white
2) Blurring image with GaussianBlur (kernel size = 7)
3) Finding edges with Canny
4) Masking trapezoid area for "cleanup"
5) Finding lines with HoughLinesP function

You can see it using   
![jpg pipeline](single_image_pipeline.ipynb)


### 2. Calculations

1) Slopes - left / right line  
At this point I had detected both lines (left and right) in one list/array.
I calculted slope for each line so i could distinguish right from left.
I removed some of the noise at this point by setting some limits for slopes.

2) Sorting  
To avoid any issues with drawing lines, i sorted lines by starting x

3) Averaging line  
I averaged each line to a point by calculating mean of x1+x2 and y1+y2

4) Fitting poly  
I found slope and intercept for all points detected in previous step. 

5) Finding first point   
I found it using max/min functions. 

6) New lines  
I calculated for x,y beginning and end of each line using formulat y=slope*x + intercept

7) Bottom fix  
I added "fake" point at the bottom of image to extend lines to the border of image 
even if detected line wasn't that long.

8) Temporal calculations
I calculate moving average based on up to 10 frames (i don't use "future" values, only "past").

### 2. Potential shortcomings 

I think there are two main shortcoming of this algorithm
- It would work poorly if there are any cars in front of camera. 
- It wouldn't work at all, on regular roads (not highways) with much tighter curves 
(Polyfit would need to be changed to allow creating curved lines).


### 3. Possible improvements to your pipeline

Potential improvement could be to set upper boundary to make line even more stable. 
Another idea would be to add higher level polyfit to make lines curved at the end.
