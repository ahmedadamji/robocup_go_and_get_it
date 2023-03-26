# RoboCup@Home LASR Team: Go and Get It Task (2021)
This repository contains the code and documentation for the successful implementation of the "Go and Get It" task in RoboCup@Home by the LASR research group in 2021. This was the team's first participation in the competition.

## Introduction
The "Go and Get It" task focuses on providing a person with something to consume when requested. The robot must navigate through a messy environment, identify the requested item, grasp it, and deliver it to the correct person.

Our implementation achieved high scores in all aspects of the task, including navigation, object recognition, grasping, and human-robot interaction. This README provides an overview of the task, our approach, and the scores achieved.

## Task Overview
The task consists of two main parts:

- Navigating to a location in Room 2 while avoiding random obstacles lying on the floor within the access area.
- Grasping the requested food item from the shelf, then delivering it to the correct person in the delivery area.
- For a detailed description of the task, please refer to the problem statement provided above.

## Approach
We utilized a combination of state-of-the-art algorithms and custom solutions to accomplish the task:

- Navigation: We used SLAM (Simultaneous Localization and Mapping) to create a map of the environment and path planning algorithms to navigate around obstacles.
- Object Recognition: Our object recognition module leveraged deep learning techniques to identify the requested item among various food items.
- Grasping: We developed a custom grasping strategy to handle occluded objects in the shelf, which involved moving other items to access the target object.
- Human-Robot Interaction: Our solution used a combination of computer vision and natural language processing to identify the correct person to deliver the item to and understand their request.
## Results
Our implementation achieved high scores in the competition:

- Navigating to a location in Room 2 while avoiding random obstacles: 100 points
- Grasping any food item in the shelf: 30 points
- Correctly taking the requested food item: 70 additional points
- - Delivering the object to a person in the delivery area: 30 points
- Correctly detecting a person request and giving the food item to them: 70 additional points
- Correctly finishing the task within the time limit: 50 points
- 20 points per minute for any remaining time

We also minimized penalty points by avoiding collisions, false deliveries, and adhering to the 4S philosophy (Speed, Smooth/Smart, Stable, Safe).

## Getting Started
Repository
Clone the repository:

```
git clone https://github.com/ahmedadamji/robocup_go_and_get_it.git
```

## Docker Setup
This project is meant to be used inside a Docker container. Follow the instructions in the TMC_WRS_Docker repository to set up the Docker environment.

Refer to the HSR documentation here for information about the Human Support Robot (HSR).

## Running the Code
Once the Docker environment is set up, follow these steps to run the code:

- Start the simulator with docker-compose up (or docker-compose -f docker-compose.nvidia.yml up if using an NVIDIA GPU).
- Open the simulator's screen at http://localhost:3000, the IDE at http://localhost:3001, and the Jupyter notebook at http://localhost:3002 in your browser.
- In the simulator's screen, click the play button (the right-facing arrow in the lower left of the screen) to start the simulation.

In the IDE terminal, input the following command to start RViz:
```
rviz -d $(rospack find hsrb_rosnav_config)/launch/hsrb.rviz
```
RViz will appear in the simulator's screen. Click "2D Nav Goal" in RViz and set the autonomous movement goal for the HSR to move to the desired location.

## Usage
After starting the simulator and setting up RViz, you can execute the main code for the "Go and Get It" task. The code will guide the robot through the following steps:

- Navigate to the shelf in Room 2 while avoiding obstacles.
- - Identify and grasp the requested food item.
- Deliver the item to the correct person in the delivery area.
- To run the code, execute the following command in separate IDE terminals:

```
roslaunch robocup_go_and_get_it support_modules.launch
```

```
rosrun robocup_go_and_get_it state_machine.py
```

Monitor the robot's progress in the simulator's screen and RViz.

## Customization
You can customize various aspects of the task, such as the number of objects, their categories, and the environment setup, by modifying the appropriate configuration files in the config folder.

## Authors
LASR Research Group (2021 RoboCup@Home Team)

## Acknowledgments
We would like to thank RoboCup@Home for providing the competition platform and the TMC_WRS_Docker repository for the Docker setup and HSR documentation.
