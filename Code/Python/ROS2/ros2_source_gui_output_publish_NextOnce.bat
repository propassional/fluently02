call C:\dev\ros2_humble\local_setup.bat
ros2 topic pub /gui_output std_msgs/msg/String "{data: '{\"transcription\": \"Next\", \"timestamp\": 1}'}" --once