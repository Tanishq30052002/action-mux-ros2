<h1 align=center>
ROS 2 Action Mux</br>
</h1>
<h2 align=center>
Google Summer of Code [Assignment - PAL ROBOTICS]
</h2>

## Introduction

- Write a basic code in ROS 2 writing an action server and the action client. However, the service request is received by subscribing a topic. If the topic is published at higher rate, it should be able to abort the previous goal and send a new one. The action server should wait for 5 seconds before succeeding and any abortion within 5 seconds should be properly handled

- Write a generic subscriber that can subscribe to a topic of any message type. Write different publishers to test the code, whenever it receives a message from a topic, print also the type of message that is received

## Table of Contents

- [Introduction](#introduction)
- [Table of Contents](#table-of-contents)
- [Setup](#setup)
- [Usage](#usage)
- [Code Structure](#code-structure)

## Setup

### Building the Project

- Follow the steps to build the project
  ```bash
  mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
  git clone https://github.com/Tanishq30052002/action-mux-ros2.git

  <!-- Third Party Lib from OSRF -->
  git clone https://github.com/osrf/dynamic_message_introspection.git

  cd ~/ros2_ws
  colcon build

  <!-- Add the following command in you setup.zsh/setup.bashrc -->
  source ~/ros2_ws/install/setup.bash   ## For bash users
  source ~/ros2_ws/install/setup.zsh    ## For zsh users
  ```

## Usage

### Running the Project

- Follow the steps to run the TASK - 1
    ```bash
    <!-- Terminal - 1 (Action Server) -->
    ros2 run calculator run_server_calculate

    <!-- Terminal - 2 (Action Client) -->
    ros2 run calculator run_client_calculate ### without feedback printing
    ros2 run calculator run_client_calculate --ros-args --log-level calculator_action_client:=DEBUG ### with feedback printing

    <!-- Terminal - 3 (Goal Publisher) -->
    ros2 run calculator run_calculator_goal_publisher <TIME_DIFF_BETWEEN_2_GOALS in secs>
    ```

- Follow the steps to run the TASK - 2
    ```bash
    <!-- Terminal - 1 (Generic Subscriber) -->
    ##### TO RUN GENERIC SUBSCRIBER IN PYTHON  (RECOMMENDED) #####
    ros2 run generic_subscriber_py run_generic_subscriber

    ##### TO RUN GENERIC SUBSCRIBER IN C++  (WIP) ####
    ros2 run generic_subscriber run_generic_subscriber

    <!-- Terminal - 2 (Testing Publisher) -->
     ros2 run generic_subscriber run_testing_publisher

    ```

## Code Structure

The project is organized into several directories:

- TASK - 1
    - `calculator_msgs` ROS2 pkg which contains the `custom msgs` and `custom actions` used for action client and action server implementation
      - `action` Custom action for the ROS2 action.
      - `msg` Custom msg for the ROS2 action and Goal Publisher.
    - `calculator` ROS2 pkg which contains the ROS2 action implementation and goal publisher code.
      - `calculator` Module responsible for the creation of the actions and publishers
      - `src` Final codes responsible for executing the code.

- TASK - 2
    - **Python Implementation** - **Recommended**
        - `generic_subscriber_py` ROS2 pkg which contains python code for generic subscriber
            - `generic_subscriber_py` Module responsible for the creation of the generic subscriber in python

    - **C++ Implementation** - **WIP**
        - `generic_subscriber` ROS2 pkg which contains C++ code for generic subscriber and testing publisher
            - `generic_subscriber` Module responsible for the creation of the generic subscriber and testing publisher in c++
            - `src` Final codes responsible for executing the code.
