# NoFire Squad - SARC 2022

## Introduction

In the modern world, working with a swarm of drones is gaining increasing prominence in research and  academic work due to its great potential for applicability. However, even though it is becoming more  common, it still presents a high level of complexity due to the challenges of controlling and coordinating  the movements of several UAVs at once. This project aims to use the swarm aproach to create a solution to a forest fight scenario, which will be simulated using the Gazebo environment and ROS services. Using 

<p align="center">
  <img width="500" height="281" src="./assets/simulation_starting.gif">
</p>

## How it works

The NoFire Squad works with the use of a swarm of 9 independent uavs that communicate with a central node, which performs all the definitions of positions and trajectories for each of the drones. In addition, one of these drones, determined as the central drone, acts only in the supervision and dimensional definition of the area affected by the fire, while the other aircraft are responsible for direct firefighting. In addition, the logic addressed for fighting the fire was to use a state machine for the swarm, in order to differentiate the decisions made according to the current state.

<p align="center">
  <img width="700" src="./assets/State_machine.png">
</p>

## Getting Started

This github is presented as a ros package, using as a base the package presented by SARC ([github](https://github.com/2nd-sarc-barinet-aerospace-competition/sarc_environment)), in order to allow a single installation of the environment proposed by them and the implementation discussed above.

### Dependencies

- Ubuntu 20.04
- Gazebo 11.10
- ROS Noetic
- MRS System
- NOTE: MRS documentation [https://ctu-mrs.github.io/](https://ctu-mrs.github.io/)

### Installing

### Executing simulation

## Conclusion

### NoFire Squad Team Members

- Daniel Yukio Miguita: [Github](), [Linkedin]()
- Guilherme Barela de Castro: [Github](), [Linkedin]()
- Guilherme Barros Villela: [Github](), [Linkedin]()
- Lucas Harim Gomes Cavalcanti: [Github](), [Linkedin]()
- Matheus Della Rocca Mastins: [Github](), [Linkedin]()
- Raul Cotrim Ferreira: [Github](), [Linkedin]()
