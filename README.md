# roche-robotics
This is *Roche* group doing the COM-304 Communications project in robotics.

## Load the model onto a TurtleBot 4 Lite
Before anything, have a stable connection to a TurtleBot 4 Lite.
1. Put the file `ppo_random_ball_and_robot_pos_rgb_v01.zip` at the same level as the files `rl_agent_controller.py` and `robot_action_executor.py` (find them [here](https://github.com/francelu/roche-robotics/tree/main/load)).  
2. In `rl_agent_controller.py`, change the `MODEL_PATH` (line 13) to the right one on your computer of the PPO model (`ppo_random_ball_and_robot_pos_rgb_v01.zip`).  
3. Install the required Python packages for Stable Baselines3:
    ```bash
    pip install stable-baselines3[extra] opencv-python numpy
4. Open two terminals and source in both the following:
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```
5. Go to the same level as where the files `rl_agent_controller.py` and `robot_action_executor.py` are.
6. In the first terminal, run:
    ```bash
    python3 robot_action_executor.py
    ```
7. In the second terminal, run:
    ```bash
    python3 rl_agent_controller.py
    ```
8. (Optional) If the robot were to undock and move backwards into the dock, in the file `rl_agent_controller.py`, you could toggle the line 33, undock it by hand and move the robot some other place before steps 6 and 7.