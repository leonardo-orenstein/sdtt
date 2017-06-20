# sdtt
An autonomous vehicle implementation

I case anyone steps by. The goal of this project is build an autonomous vehicle framework, mostly as a learning enviroment. The idea would be the provide the segmented infrastructure, whith each individual module being independent (i.e.: the controller is agnostic to what sensor is being used). Right now is running on simulation alone, but the idea would be to have it connect to hardware (the donkey project being the prime candidate - http://www.donkeycar.com/)

# Getting started
Just clone the code and run the run_through_track.py or the jupyter notebook (although I think the animation runs better on the first).

# Want to help?
Any help is welcomed! From ideas to coding just drop me a message!

ToDo:
- Add uncertanty in Sensor
- Add error/slack in Actuator
- Add MPC
- Add Kinematic model
- Add more complex tracks
- Add vision component (Sensor?)
- Add CNN controller (future work)
- HW
