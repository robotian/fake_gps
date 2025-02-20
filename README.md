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



