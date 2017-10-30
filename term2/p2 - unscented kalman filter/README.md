## Unscented Kalman Filter Project

### How to run:
cmake CMakeLists.txt  
make  
./UnscentedKF  

### Results:
My final RMSE results:  
![Final RMSE](images/rmse.png)  

You can see it running here:  
https://vimeo.com/240435221  
password: udacity  

### Comments:
1) At first I copied lesson parts into project and changed variable names into "_" versions. 

2) Project overview video helped here to fix minor issues with code.   

3) I forgot to fill x and P with zeros in Prediction and had "partly" working solution but I found it, by watching P values going higher and higher. You can see all data displayed in ukf_debug version of this file.   

4) The last issue was finding right parameters - after playing a while with std_a and std_yawdd I got closer to proper RMSE values but it wasn't enough. I had to change a little bit init x values to reach RMSE target.   


 
