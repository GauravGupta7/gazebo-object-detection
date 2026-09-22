# gazebo-object-detection

## Execution Commands:
```bash

# For Camera World launch
roslaunch object_detection_gazebo camera_world.launch

# For running scripts
rosrun object_detection_gazebo color_object_detector.py

# Camera live feed
rosrun rqt_image_view rqt_image_view
```

## 3D Projection and Camera Intrinsics

### *Concept*
Now, since our robot has percepted the environment in the form of images which contains pixels localized using 2D coordinates $(u,v)$, we need to transform it to real world 3D coordinates $(X_c, Y_c, Z_c)$ relative to the camera and world frames. 

A 3D point in space is transformed to 2D image using the camera intrinsic matrix $K$. The matrix $K$ is composed of focal lengths $(f_x, f_y)$ and the optical centre $(c_x, c_y)$. The pinhole projection equation for 2D image generation is given by: 

$$
\begin{bmatrix}
    u \\\\
    v \\\\
    1
\end{bmatrix}
=
\frac{1}{Z}
\begin{bmatrix}
    f_x & 0 & c_x \\\\
    0 & f_y & c_y \\\\
    0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
    X_c \\\\
    Y_c \\\\
    Z_c
\end{bmatrix}
$$

To invert this projection from 2D to 3D, given the object depth $Z_c$, we back project to find the 3D spatial position $(X_c, Y_c)$

$$
X_c = \frac{(u-c_x) Z_c}{f_x}
\\
Y_c = \frac{(u-c_y) Z_c}{f_y}
$$

Thus, obtaining the spatial coordinates $(X_c, Y_c, Z_c)$

### *ROS implementation*
Gazebo publishes the intrinsic parameters $(f_x, f_y, c_x, c_y)$ details on the topic `/camera/camera_info`. 

Since our camera current topic `/camera/rgb/image_raw` is an RGB stream, we can estimate depth from known geometry or switch to an RGB-D camera. 