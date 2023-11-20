
# Launch Files

## awl_wheelchair launch file
```
ros2 launch luci_third_party wheelchair.launch.py
```


Launches
- grpc interface node (from luci_grpc_interface)
    - uses flag -a to set the chair ip address, which is 192.168.8.203
- luci tf node between base_link and all the sensors (from luci_transforms)
    - transforms are for the quickie 500m wheelchair but can also be for permobil m3

## awl_description launch file
```
ros2 launch luci_third_party awl_description.launch.py
```

Launches 
- joint state publisher (publishes the joints from the xacro file)
- robot state publisher (publishes the description of the robot)
- rviz2 using the rviz config to see the robot description topic