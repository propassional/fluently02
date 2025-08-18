Mode: EBuddy with ROS2
Sockets act as separation layer between EBuddy environment and ROS2 environment, that are kept separate, being enough complicated
1. Run EBuddy with ROS2 mode active
2. Open FLROS2ListenerSocket.py in pycharm, set breakpoints if wished
3. Run FLROS2ListenerSocket.py in pycharm terminal: >C:\Python38\python.exe FLROS2ListenerSocket.py
   C:\Python38\python.exe is pre-sourced, there is no need to source it
4. You should see each state change in the listener

Mode: Cmd and pycharm terminal
# 1. Open cmd, >C:
# 2. Source ROS2 with >call C:\dev\ros2_humble\local_setup.bat
#     Answer is (which is ok): "[rti_connext_dds_cmake_module][warning] RTI Connext DDS environment script not found (\resource\scripts\rtisetenv_x64Win64VS2017.bat). 
      RTI Connext DDS will not be available at runtime, unless you already configured PATH manually."
# 3. In cmd: >ros2 run demo_nodes_cpp talker
#    You will see the hello world messages appearing each second
# 4a. Run python code from the pycharm terminal, from the directory where it resides, and you will see the ROS communication happening: >C:\Python38\python.exe FLROS2Listener.py
#     Remark that C:\Python38\python.exe is pre-sourced, there is no need to source it
	  Test: >C:\Python38\python.exe --version
# 4b. Run python code from cmd: open cmd, run >C:\Python38\python.exe D:\Banfi\Github\Fluently\Code\Python\ROS2\FLROS2Listener.py

Processing chain:
EBuddy starts with SOCKET_SERVER_IS_ON = True, a socket server is launched, and at each state change, the state is sent via socket
FLROS2Talker.py reads some valid commands from the intents doc (Next, for example), and sends them via ROS2 (fake commands)
FLROS2ListenerSocket.py listens the fake commands, sends them via socket