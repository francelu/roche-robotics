# roche-robotics
This is *Roche* group doing the COM-304 Communications project in robotics.

## Repository Structure

- **`load/`** – Contains the main agent controller and executor files used to run the trained PPO model on the TurtleBot.
- **`Roche_Robotics.ipynb`** – Jupyter Notebook that documents the training process, model architecture, environment setup, and evaluation results for the PPO agent.
- **`requirements.txt`** – Lists all Python dependencies needed to run the training, evaluation, and deployment scripts, including stable-baselines3, habitat-sim, gymnasium, and visualization libraries.

## Using the `Roche_Robotics.ipynb` Notebook

The Jupyter Notebook `Roche_Robotics.ipynb` provides a detailed overview of the entire project, including:

- The training process of the PPO agent, with explanations on the environment setup and hyperparameters.  
- How the simulation environment was configured and run.  
- Evaluation metrics and results demonstrating the agent’s performance.  
- Code snippets for training and evaluation to facilitate reproduction and understanding.  

To use the notebook:

1. Access the Izar cluster and perform the same setup as described in [Turtlebot4 setup guide](https://github.com/EPFL-VILAB/com-304-robotics-project/blob/main/Turtlebot4_setup/Turtlebot4_setup_guide.md).  
2. Add the notebook to your workspace at the same level as `RL_Habitat_Homework/RL_Habitat_Homework.ipynb` or select the kernel used by that file.  
3. Place the target files `.glb` and `.object_config.json` in `/workspace/habitat-sim/data/test_assets/objects/` or update the paths in the notebook accordingly.  
4. Open the notebook with Jupyter Notebook or JupyterLab:
    ```bash
    jupyter notebook Roche_Robotics.ipynb
    # or
    jupyter lab Roche_Robotics.ipynb
    ```
5. Install the required Python packages listed in `requirements.txt` using:
    ```bash
    pip install -r requirements.txt
    ```
6. Run the cells sequentially to explore the training pipeline and evaluation.  
7. Adjust parameters or experiment with the code to better understand or improve the model.  

This notebook is a valuable resource both for reproducing the training and for understanding the design decisions behind the PPO agent deployed on the TurtleBot.

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
