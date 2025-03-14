# fake_gps
ROS2 node to calculate the X-Y translational displacement from a reference frame to a target frame, convert it to Longitude and Latitude values, and publish it as NavSatFix message topic. 

Coordinate System: WGS84

UTM Zone: 16 North

- Longitude: X axis displacement
- Latitude: Y axis displacement


## Parameters
- base_tf (default: 'map' ): the reference TF 
- target_tf (default: 'base_link'): the target TF
- gps_tf (default: 'gnss_link'): GPS link id


## Usage

```
ros2 launch fake_gps fake_gps.launch.py 
```
Launch parameters for the node

```
parameters=[
                {'use_sim_time':False},
                {'base_tf':'map'},
                {'target_tf':'base_link_gt'},
                {'gps_tf':'jackal_gnss_link'},      
            ]
```

## To Do
- Noise Parameter
- Covariance matrix 
- Orientational diffencen between map tf and UTM


## Installation

Clone the repository into your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/fake_gps.git
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select fake_gps
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Running the Node

To run the `fake_gps` node with default parameters:

```bash
ros2 launch fake_gps fake_gps.launch.py
```

To run the node with custom parameters:

```bash
ros2 launch fake_gps fake_gps.launch.py base_tf:=<your_base_tf> target_tf:=<your_target_tf> gps_tf:=<your_gps_tf>
```

Replace `<your_base_tf>`, `<your_target_tf>`, and `<your_gps_tf>` with your desired frame IDs.

## Example

To run the node with example parameters:

```bash
ros2 launch fake_gps fake_gps.launch.py base_tf:=map target_tf:=base_link_gt gps_tf:=jackal_gnss_link
```

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
