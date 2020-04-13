imu_estimator
====
![CI](https://github.com/rsasaki0109/imu_estimator/workflows/CI/badge.svg)  
a header-file program of attitude estimator by using 6-axis imu with gyro bias correction
## how to use

```cpp
#include <imu_estimator/ekf.hpp>

EKFEstimator ekf;
ekf.setProcessNoize(0.033); // [(rad/sec)^2]
ekf.setObservationNoize(0.033);// [(m/sec^2)^2]
// input
dt = 0.1; // [sec]
Eigen::Vector3d gyro{0, 0, 0};
Eigen::Vector3d acc{0, 0, 0};
// output
Eigen::Quaternion<double> quat;

ekf.filterOneStep(quat, dt_imu, acc, gyro);


```

## ros2 dashing example
- input  

/imu/data  (sensor_msgs/Imu)(gyro and acc)

- how to run

```
ros2 run imu_estimator imu_estimator 
```

