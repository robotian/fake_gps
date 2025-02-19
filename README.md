# fake_gps
Calculate the X-Y translational displacement from a reference frame to a target frame, convert it to Longitude and Latitude values, and publish it as NavSatFix message topic. 


## Parameters
- base_tf: the reference TF 
- target_tf: the target TF
- gps_tf: GPS link id

## Usage

```
ros2 launch fake_gps fake_gps.launch.py 
```


