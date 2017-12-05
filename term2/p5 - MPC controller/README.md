# Udacity CarND MPC Controler 


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

		f0 = coeffs[0] + coeffs[1] *x0 + coeffs[2] * x0 *x0 + coeffs[3] * x0 *x0 *x0;  
		psides0 = atan(3*coeffs[3]*x0 * x0 + 2*coeffs[2]*x0 + coeffs[1]);  

		x = x0 + v0* cos(psi0) * dt;  
		y = y0 + v0* sin(psi0) * dt;  
		psi = psi0 - v0 * delta0 / Lf * dt;  
		v = v0 + a0 * dt;  
		cte  = ((y0-f0) + (v0 * sin(epsi0) *dt));
		epsi = ((psi0 - psides0) + v0 * delta0 / Lf *dt);


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
I tested time steps from 0.1 to 0.2. When time step was small (0.1s), model worked with maller speeds but above 70mph it wasn't working properly.  I also tried 0.1 with 15 steps but 0.18 with 10 steps worked much better. 


## Polynomial fitting and mpc processing
I used method mentioned in the walkthrough video where you assume that car position is 0,0 and turn (psi) is also 0. Using those assumptions, you transform points given by the simulator to be in "car coordinates system". This step simplifies next calculations. Then you fit a 3rd degree polynomial using those points. Calculated coefficients are stored in "coeffs" variable. 


## Full general algorithm
1) Read information from the simulator
2) "Zero" car coordinates
3) Transform 6 waypoints to car coordinates
4) Calculate polynomial
5) Incorporate latency
6) Set "State" variable
7) Use solver to get steering and throttle
8) Send steering and throttle to simulator
9) Go to step 1


## MPC with latency
There were two ways to incorporate latency - before calculating points and after. First method is in theory more accurate but second worked properly. 

To incorporate latency I used equations:
		
		double delta = steer_angle/ (deg2rad(25) * Lf);  
		px = v*latency;  
		py = 0;  
		psi = -v*delta*latency/Lf;  
		double epsi = -atan(coeffs[1]) + psi;   
		double cte= polyeval(coeffs,0)+v*sin(epsi)*latency;  
		
I added Lf factor in the first equation - because without it, car was swinging from right to left (predicted angles were too big). I didn't change speed because there is no way to tell real accelaration of this car - so I didn't use the line below:
//v += a*latency;
     


## Remarks:
1) It is much easier to create solution that works during first lap, than the solution that works for at least 2 laps.   
2) With timestep of 0.1s it wasn't possible to reach 100mph and make it stable enough.  
3) Visualizations have some issues with displaying green/yellow line. I fixed it by adding manually another point for green line.   
4) I converted MPH to m/s by using 0.44 multiplier (without it solution wasn't working properly).  
5) I used method of "zeroing" position of the car suggested in the project walkthrough video.   
6) As suggested in one of the forum threads I used (y0-f0) instead of (f0-y0).  
7) There is no real way to tell, what is the acceleration of the car (throttle is just some weird approximation).  







