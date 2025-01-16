# Mission Control Interface

A versatile Control Interace for the Cardinal Space Mining Club using ROS2-jazzy, ImGui, SDL2, and OpenCV. Originally, this project started for personal development, practice, and testing and evolved into a functional application to be used in conjuntion with the Cardinal Space Mining Club robot. Used to manage the robotic system (disabling and selecting operating mode) and to visualize data in real time.

Note: This is developed for Ubuntu Linux. CMake is designed for Unix only, ROS2 jazzy, SDL, and OpenCV required for building.

**Key Features**
- Real-Time Video Feed: Display live video streams from robotic system and select which feeds to stream.
- 3D LiDAR Visualization (in progress): View and manipulate 3D LiDAR point clouds
- Customizable Interface: Intuitive GUI layout using ImGui and ImPlot with SDL for rendering
- ROS2 Integration: Integrated with the Robot Operating System (ROS) for seamless communication and management of sensor, camera, and control data messages and commands
- Data Processing: Use of OpenCV to efficiently process and display image data

**Building and Running**

`cd ControlInterface`

`colcon build <--executor parallel> <--cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON>`

`source install/setup.bash`

To run Mission Control Interface:
`ros2 run gui_interface main`

To publish camera footage/images use:
`ros2 run camera_test main`

To view camera footage being published without application use:
`ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw`

For statistic plots to populate with data use:
`ros2 run info_gen main`
