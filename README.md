# Tactile-Based Upper Limb Motion Correction System

- [Barry Zheng](https://github.com/wxzheng25)
- [Chang Liu](https://github.com/fluencycy)
- [Richal Yu](https://github.com/Richal13Yu)
- [Xinyu Wang](https://github.com/wxy02111)

## Project Demo Video

[![YouTube](http://i.ytimg.com/vi/wX_SBOqtOxI/hqdefault.jpg)](https://www.youtube.com/watch?v=wX_SBOqtOxI)

This device can be divided into three main components. The first is a control and data. The second is the upper-arm haptic system, which includes an inertial measurement unit (IMU) and four vibration motors. The IMU works in conjunction with another IMU on the forearm to compute the overall arm gesture in Cartesian space.

The user wears a tactile-based band on the upper limb. By following the haptic guidance, the user is expected to replicate the same limb gesture to learn the movement. The haptic armband is equipped with vibration modules, while the device integrates two IMUs attached to the upper arm and forearm. These sensors capture the unique curvatures and range of motion of the upper arm and forearm during various actions.

The haptic system utilizes four vibration motors on the armband. Each motor provides guidance for the upper limb's movement in 3 degrees of freedom (DOF): up/down, left/right, and twisting. For twisting, the motors vibrate in a specific sequence to guide the arm's rotational motion.
The guidance process follows three steps:
1.	Whole-Arm Calibration: The system activates the horizontal and vertical vibration motors to guide the user in calibrating the entire arm's position.
2.	Forearm Rotation: The vibration motors on the forearm operate in a specific sequence to indicate the required twisting motion.
3.	Elbow Joint Adjustment: The inner-side vibrator on the forearm guides the user to achieve the correct elbow-joint angle.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/35b02e44-d6c7-47bc-a3b5-3325a9307232" />
<img width="400" alt="image" src="https://github.com/user-attachments/assets/02e58061-b05a-441b-8dec-91f779973e2e" />

Firstly, we calibrate the whole arm position by using horizonal and vertical vibrators to guide. Secondly, the vibrator set at forearm rotates in a certain sequence to indicate the twist. Finally, the vibrator of the forearm inside will guide the forearm to the correct elbow-joint angle.

<img width="400" alt="image" src="https://github.com/user-attachments/assets/bff8b5e7-de8a-4965-bfae-56e841e9214c" />
<img width="400" alt="image" src="https://github.com/user-attachments/assets/813544a0-e8bf-4e9e-aedd-c9b9494707e9" />
