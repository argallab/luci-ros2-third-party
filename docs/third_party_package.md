# luci-third-party

## Wheelchair launch file

After correctly sourcing ROS2, run the following to start the node:
```
ros2 launch luci_third_party wheelchair.launch.py ip:={Chair IP address}
```

This launch files starts up the gRPC interface node and transformation node. The ip launch argument is used to specify the specific chair to connect to. 