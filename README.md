# CarND-Kidnapped-Vehicle
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

This is the project repository for **Project No. 3 Kidnapped Vehicle**, part of Term 2 _Sensor Fusion_ of Udacity Self-Driving Car Nanodegree program, submitted by Albert Killer in June 2017. 

A 2 dimensional Particle Filter was implemented in C++. The algorithm is given some initial localization information (analogous to what a GPS would provide) and a map including the position of landmarks on an arbitrary Cartesian coordinate system. At each time step the filter will also get observation and control data. The Particle Filter then localizes the vehicle's position and yaw to within the values specified in the parameters *max_translation_error* and *max_yaw_error* in *src/main.cpp* and is able to complete execution within 100 seconds. 

To evaluate the results *main.cpp* communicates with Udacity's *Term 2 Simulator* using *uWebSocketIO* :

- [x] This is a complete item
- [ ] This is an incomplete ite

![Evaluating results using Udacity's Term 2 Simulator](Screenshot%20from%202017-06-10%2017-35-51.png?raw=true "Evaluating results using Udacity's Term 2 Simulator")


Want to know more about Particle Filters or Sequential Monte Carlo (SMC) methods? Have a look at this one: https://en.wikipedia.org/wiki/Particle_filter
