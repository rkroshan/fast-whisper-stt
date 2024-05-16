### how I created the ros2 package
`mkdir -p ros2_ws/src`
`cd ros2_ws`
`ros2 pkg create --build-type ament_python --license Apache-2.0 py_srvcli --dependencies rclpy py_srcvli`

### build the package
`cd ros2_ws`
`colcon build --packages-select py_srvcli`

### run the server
`cd ros2_ws`
`ros2 run py_srvcli service`

### run the client
`cd ros2_ws`
`ros2 run py_srvcli client`
#### client is similar to test_record_client.py
