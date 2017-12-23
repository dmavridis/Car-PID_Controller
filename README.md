# Project 4: PID control 

## Introduction
There are two versions of the code that I would like to submit. 

In order to compile a file, the easiest way is to rename the `main_x.cpp` file to `main.cpp`

## Impementation Details
The first one, implemented in the file `main_1.cpp` is a straightforward approach  where the PID control is implemented and the throttle is constant at 0.3. The main challenge in this case is the identification of the right coefficients. I applied the values for $K_p,K_i,K_d$ from the lectures and it is working reasonably well. Experimenting with the various parameters, initially only P control was applied and progressively PD and PID. The results are recorded in the video [Video](https://www.youtube.com/watch?v=_r4vA4SPNvs)



- **P only**: The car fails and gets out of the road. P only is not enough
- **PD**: Can behaviour is correct and it is driven successfully
- **PID**: Correct behaviour. It is hard to tell the difference between last 2 cases. Ki component is very small to affect significantly. 

##  Twiddle
Next step is to add twiddle, which is implemented in the file `main_2.cpp`.That task was more challenging as the initial implementation was overloading the simulator and there were robustness issues. The problem was that when calculating the coefficients and a measurement is required to check whether the error is smaller, there is not enough time for the simulator to get the latest data. 

The implemented code is using global variables to allow a better flow of the programm. The variables meas1 and meas2 are checking whether the first or the second measurement of the twiddle algorithm are executed.

The result is giving coefficients that are converging to similar values of the first approach, based on the lectures. However, experimenting with the initial conditions, it was proven that the coefficient final values are highly dependent on initial conditions and instability can occur. To add an extra safety, the twiddle part of the code is execured when the speed is above a specific value. Otherwise the error at very low speeds can be low causing the coefficients to have very different values. 

For the speed, a simple control mechanism that is adjusting the speed according to the total error and the steering value is implemented. The parameters are chosen after several sets of experimentation. 

## Issues
The twiddle implementation is leading to satisfactory performance but if the speeds need to increase significantly failure can occur. Increasing the speed coefficient will therefore require larger negative coefficients for the steering and the error. The overall result will not change massively if the coefficients are increased proportiinally. 


