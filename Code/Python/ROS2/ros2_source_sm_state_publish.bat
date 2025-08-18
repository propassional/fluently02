call C:\dev\ros2_humble\local_setup.bat
ros2 topic pub /sm_state std_msgs/msg/String "{data: 'InstructionsEnd'}"