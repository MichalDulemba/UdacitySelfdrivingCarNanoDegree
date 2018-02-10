# CarND Path Planning Project
   

### Goals
Main goal of this project was to drive safely on highway. Change lanes when safe, possible and when it makes sense. One lap is around 4miles and I should complete it in around 5mins.
You can see my results at:  
https://vimeo.com/255140742  
pasword: udacity  


### My approach
As a code base I used ideas from the walkthrough video. Especially using spline to "draw" a path was useful. But I would say that it is 30% of the final project
- speed control and all logic for changing lanes was done from scratch. 

Some assumptions:
- I cannot see car that is more than 100 m away 
- I use all lanes equally, while in real it would be better to overtake using only left lane (relative to other car)
- for speed control I used couple of PID controllers tested with different coefficient values 
- I will make this car drive in a similar way to my driving when i don't want to get ticket but need to be on time ;)
- To avoid any confusion during testing, i used "hard" logic, instead of "fuzzy" cost function. 


### Changing lane model/logic

- if there is no car, in front of me - drive at max speed, don't change lane
- if I spot a car when between 100 and 50m to his position - find empty adjescent lane and change if it's safe 
- if there is a an empty lane but it is not adjescent - check if you can switch to closer lane (middle one in this case) - change if it's safe
- same for closer distance but a little different spline settings
- if there is no empty lane - check for faster lane - change if it's safe
- if there is no empty or faster lane - keep this lane
- adjust the speed to avoid hitting car in front of me

Checking if maneuver is safe is based on looking if certain range on the target lane is empty. This way I avoid hitting other car with my door, and getting hit from behind.


### Difficulties
Tuning PID - I had several issues. At first i lost probably 2 days trying to tune it, while using mph for my car, and m/s for the target speed. Finally i found reason for crazy PID behaviour. But it didn't help with cars in front of me changing speed harshly or with crazy drivers appearing from other lanes. Finally I splitted PID behaviour based on the distance to the car in front of me. So the closer ego car gets to the next car on this lane, the more I need to break.


One of the biggest challanges for me, was to write "running from local max". What I mean is the situation, when the best possible lane is not the adjescent one, and you need to do 2 lane changes (or one long one). It added lots of different situations that needed to be handled.


Even with really complicated logging, sometimes it was really hard to say, what really happened. Therefore from some point, I recorded all driving with screen capture. 
It helped greatly to find "bugs" in the system. 

### Additional "challange" 
I added 10 s waiting to make traffic more dense in front of the ego car, to simulate more difficult situations. 


### What can be improved
I'm guessing this can be improved almost indefinitely - because of the number of possible situations. 
Also after long nights and mornings - there is a lot of refactoring to be done (for example extracting PID). But first "make it run", later "make it nice".
It would be nice to add simple "memory" and remember other cars information for 10 cycles. This would allow to predict some crazy moves like unexpected lane changes.






