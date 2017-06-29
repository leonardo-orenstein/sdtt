# sdtt
An autonomous vehicle implementation

<p align="center">
  <img src="\truck_demo.gif" width="350"/>
</p>

In case anyone steps by. The goal of this project is build an autonomous vehicle framework, mostly as a learning enviroment. The idea is to a segmented infrastructure, where each individual module is independent (e.g.: the controller is agnostic to what sensor model is being used and how it was implemented). Right now is running on simulation alone, but the plan is to have it connected to a hardware (the donkey project being the prime candidate - http://www.donkeycar.com/)

# Getting started
Just clone the code and run the run_through_track.py or the jupyter notebook (although I think the animation runs better on the first).

# Want to help?
Any help is welcomed! From ideas to coding just drop me a message!

ToDo:
- Add uncertanty in Sensor
	> Done (06/20)
- Improve performance of the Particle Filter/Simulation/Animation
- Add uncertanty/slack/quantization in Actuator
- Add SLAM
- Improve plots
	> Done for now (06/22)
- Add MPC
- Add Kinematic model
- Add more complex tracks
  > Track class with pre built components?
  	> Track Class done for now (06/22)
- Add vision component (Sensor?)
- Add CNN controller (future work)
- HW
