# CarND-Controls-MPC


## Basic Build Instructions

1. Clone this repo.  
2. Make a build directory: `mkdir build && cd build`  
3. Compile: `cmake .. && make`  
4. Run it: `./mpc`.  


## Video - working model
https://vimeo.com/245860342  
password: udacity  
or see the uploaded file.   


## The model
We are using kinematic model here. Model is based on those equations:



## Cost function


   To create cost function for the solver i added cte, epsi, v, delta, a, delta change and a change
   with respective weight multipliers:   

   smooth_turn = 50;  
   smooth_turn_change = 200;  
    
   smooth_speed =90;  
   smooth_speed_change =10;  

   cte_weight =2000;  
   epsi_weight =2000;  
   speed_weight =1;  

   ref_v = 100;  



## Timestep Length and elapsed duration
I tested time steps from 0.1 to 0.2. When time step was small, model worked with maller speeds but above 70mph it wasn't working properly.  



## Polynomial fitting and mpc processing



## MPC with latency



## Remarks:
1) It is much easier to create solution that works during first lap, than the solution that works for at least 2 laps.   
2) With timestep of 0.1s it wasn't possible to reach 100mph and make it stable enough.  
3) Visualizations have some issues with displaying green/yellow line. I fixed it by adding manually another point for green line.   
4) I converted MPH to m/s by using 0.44 multiplier (without it solution wasn't working properly).  
5) I used method of "zeroing" position of the car suggested in the project walkthrough video.   
6) As suggested in one of the forum threads I used (y0-f0) instead of (f0-y0).  
7) There is no real way to tell, what is the acceleration of the car (throttle is just some weird approximation).  







