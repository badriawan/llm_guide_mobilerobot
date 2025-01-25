Welcome to the repository for DTETI UGM's guide robot development, which integrates ROS-LLM and Nav2 packages. This repository is forked from the original [nav2LLM](https://github.com/sachinkum0009/nav2gpt).

To explore detailed information about:
- **Nav2-LLM Integration**: Visit the [ROS-LLM](https://github.com/Auromix/ROS-LLM)  [nav2LLM](https://github.com/sachinkum0009/nav2gpt).
- **Nav2 config**: Refer to the [Nav2 ROS Wiki](https://navigation.ros.org/).

---

## ✨ Quickstart Guide

### 1. Fork and Clone the Repository
First, fork this repository into your own GitHub account. Then, clone it to your local machine using the command below:

```bash
git clone https://github.com/your_github_account/llm-guide-mobilerobot.git
```

```bash
cd llm-guide-mobilerobot
colcon build
```

## ⚙️ Using the Framework with Nav2

To integrate the framework with your own robot, modify the `llm_robot` and `llm_config` packages to match your robot’s specifications. This customization allows you to define the behavior and control mechanisms for your robot.

### Steps to Test with Nav2 System:
1. Source the workspace:
   ```bash
   source llm-guide-mobilerobot/install/setup.bash
   ```

2. Launch the Nav2 system:
   ```bash
   ros2 launch ros2ai navigation2.launch.py
   ```

3. Launch TurtleBot3 navigation:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch ros2ai turtlebot3_navigation.launch.py
   ```

4. Run Nav2 API Server:
   ```bash
   ros2 run ros2ai nav2_api_server
   ```

5. Run Nav2 GPT:
   Don't forget to change the .env path in load_dotenv(dotenv_path="/home/badri/nav2_gpt/.env")
   ```bash
   ros2 run ros2ai Nav2Chatgpt
   ```

7. Alternatively, test another Nav2 configuration:
   ```bash
   ros2 launch navigation navigation.launch.py
   ```

---

### Additional Notes
- Ensure all dependencies are correctly installed and sourced before launching any package.
- For troubleshooting or further details, refer to the original ROS-LLM and Nav2 documentation.

---

Happy exploring!

