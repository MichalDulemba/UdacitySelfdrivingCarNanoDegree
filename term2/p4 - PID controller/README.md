# CarND-Controls-PID

### How to run:
cmake CMakeLists.txt  
make  
./pid  

### Results:
My final (more aggresive driving / two PIDs) parameters:  

//PID for steering  
init_Kp=-0.13;  
init_Ki=0;  
init_Kd=-1.5;  

// PID for speed  
init_speed_p = -0.1;   
init_speed_d = 1;   

// throttle correction based on angle  
init_k_angle = -0.01;  

init_throttle = 0.5;  


smooth version :  
init_Kp=-0.12;  
init_Ki=0;  
init_Kd=-1.5;  

init_throttle = 0.5;  

I didn't use throttle PID here (all values set to 0)  


You can see it running here:  
https://vimeo.com/243267633  
safe version  

https://vimeo.com/243608448  
more aggresive version  

password : udacity  
or have a look at uploaded videos.   


### Comments:
1) At first I got confused by function names. Totalerror doesn't exactly sound like "CalculateSteering".  
But after having a look at Davids walkthrough it was pretty easy to get first version up and running.   

2) At this point I didn't know about simulator option "reset". So I tested parameters manually (without twiddle).  
With speed around 40-50mph after several rounds of tuning, car drives pretty stable. This is safe version of the video.  

3) When i tried to make it with much higher throttle (0.8-0.9), car almost always was driving over red lines.  
Then I added another PID for throttle. But even with this secondary PID, if you drive that aggresive on turns, car sometimes 
touches red lines / or dirt. I added another condition to make throttle more aggresive when CTE and angle is low.   
I also added my own parameter for throttle change based on steering (the harder the steer, the lower the throttle).  
I tried to make it based on the angle, but it wasn't working that well. 

4) At the end I added all statistics to make it possible to really verify if change in parameters, make it better or worse.   

5) Some ideas for further improvement would be drawing graph of car track (top view) versus "best track"   
and seeing how it really works during turns - what parameters to change to make it better.   
I was also thinking about using some kind of time series - so for example calculate parameters for maybe 10 steps and use this average
for all calculations. Maybe that would prevent from "wobbling" and gave car the information that it is during turn, so it has to keep
turning. Not sure how that would work out.   








 
 