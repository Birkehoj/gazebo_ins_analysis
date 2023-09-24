# Simulated environement for inertial navigation system

## Installation
Follow this guide to install latest gazebo simulator: https://gazebosim.org/docs/harmonic/install.
Until [issues/2163](https://github.com/gazebosim/gz-sim/issues/2163) is resolved you need to install gazebo "garden" version. 
## Build plugin

```bash
cmake -DCMAKE_BUILD_TYPE=Release -B build
cmake --build build
```
## Collect data

### Run simulation

#### Setup paths
##### Linux and MacOS
```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=${PWD}/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```
##### MacOS
```bash
export DYLD_LIBRARY_PATH=/opt/homebrew/lib:$DYLD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=${PWD}/build:$DYLD_LIBRARY_PATH
```
#### Run simulation

##### Linux
```bash
cd worlds
gz sim -v 4 loader_world.sdf
```
##### MacOS
```bash
cd worlds
gz sim -v4 -s -r loader_world.sdf
```
In another terminal run the gui
```bash
gz sim -v4 -g loader_world.sdf
```
#### Operating the simulation
When the simulator starts, the robots can be controlled with the mouse, or with keyboard depending on, which tap is selected on the right hand panel.
![gz-sim-robot-control.png](resources%2Fgz-sim-robot-control.png)
When you have moved the robot in the movement you want to analyze close the simulator.
You are now ready to analyse the data and develop your navigation algorithm.
### Simulation parameters
The [loader_world.sdf](worlds/skid-loader-635/skid-loader-635.sdf) is build based on information from here [camsind.com/en/skid-loader-635-2](https://www.camsind.com/en/skid-loader-635-2/).
The blender model [skid-loader-635.blend](worlds/skid-loader-635/skid-loader-635.blend) is based a model from [done3d.com/skid-steer-loader](https://done3d.com/skid-steer-loader/).
### The loader simulation model
To keep the simulation simple the three used sensors, IMU, GNSS and speed(gear) are placed on the same body center link.
Body center is here defined to be in the center of the wheel axes.
#### IMU noise
The default IMU noise used in loader world is defined in [loader_world.sdf](worlds/skid-loader-635/skid-loader-635.sdf).
The origin of the noise is the default values for [ADIS16448 IMU](https://www.analog.com/en/products/adis16448.html) as defined in [imu_noise_parameters.h](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/include/gazebo_imu_plugin.h).
```python
import math
kDefaultAdisGyroscopeNoiseDensity = 2.0 * 35.0 / 3600.0 / 180.0 * math.pi
kDefaultAdisGyroscopeRandomWalk = 2.0 * 4.0 / 3600.0 / 180.0 * math.pi
kDefaultAdisGyroscopeBiasCorrelationTime = 1.0e+3
kDefaultAdisGyroscopeTurnOnBiasSigma = 0.5 / 180.0 * math.pi
kDefaultAdisAccelerometerNoiseDensity = 2.0 * 2.0e-3
kDefaultAdisAccelerometerRandomWalk = 2.0 * 3.0e-3
kDefaultAdisAccelerometerBiasCorrelationTime = 300.0
kDefaultAdisAccelerometerTurnOnBiasSigma = 20.0e-3 * 9.8
update_rate = 100
dt = 1 / update_rate
stdDev = kDefaultAdisGyroscopeNoiseDensity * 1.0/math.sqrt(dt)
```
#### Forward direction speed
There is no noise on the forward direction of the loader.
#### GNSS noise
The noise on the gnss sensor is defined in [loader_world.sdf](worlds/skid-loader-635/skid-loader-635.sdf).
It is selected to match a good RTK GNSS receiver, with fixed position.
TODO: find noise parameters for gnss speed.

## Logging and playback in gazebo
To record the simulation data use the following command on linux(adapt for mac as when running above):
```bash
gz sim -v4 --record-path ./loader-log loader_world.sdf
```
To play back the data use the following command:
```bash
gz sim -v4 --playback ./loader-log
```
## Notes on background math
The Kalman filter implementation is from https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/tree/master.
### How is the speed updated in kalman filter with constant velocity assumption?
https://dsp.stackexchange.com/questions/8860/kalman-filter-for-position-and-velocity-introducing-speed-estimates
in the correct step the speed is updated based on the previous position and the predicted one.
So the position updates influences the speed.
