# Extended Kalman Filter Project Starter Code
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)

[image1]: ./img/EKF_Dataset1.PNG "EKF with Dataset 1"
[image2]: ./img/EKF_Dataset2.PNG "EKF with Dataset 2"

In this project a kalman filter is used to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

# Dependencies and Prerequisites

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

# Build and Run

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. `mkdir build`
2. `cd build`
3. `cmake ..`
    * On windows, you may need to run: `cmake .. -G "Unix Makefiles"`
4. `make`
5. `./ExtendedKF`

# Protocol between Code and Term 2 Simulator

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


**INPUT**: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


**OUTPUT**: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x

["estimate_y"] <= kalman filter estimated position y

["rmse_x"]

["rmse_y"]

["rmse_vx"]

["rmse_vy"]

---

# Results

In this simulation noisy lidar and radar measurements are given as input to the EKF (you can see them as blue and red dots in the simulator). The position estimated by the Kalman Filter is displayed as green dots.

Furthermore, there are two Datasets, where the second Dataset is just the first Dataset reversed.

In the following image are shown the results of EKF with Dataset 1.
![alt text][image1]

In the following image are shown the results of EKF with Dataset 2.
![alt text][image2]

## Accuracy

The EKF accuracies (as seen in the images above) are:

* Dataset 1: RMSE [0.0974, 0.0855, 0.4517, 0.4404]
* Dataset 2: RMSE [0.0732. 0.0963, 0.3951, 0.4730]

Both accuracies meet the requirements RMSE <= [.11, .11, .52, .52]