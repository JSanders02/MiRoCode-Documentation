# MiRoCode-Documentation
Documentation for writing Mirocode in python, using the miro2 library.
Written by Jack Sanders

# Documentation for the MiRoCODE website
At the start of any code, put the line `import miro2 as miro`

## Contents

- [MiroCode Constants](#MiroCode%20Constants)
- [MirocodeInterface Class](#MirocodeInterface%20Class)
  - [Instantiation](#Instantiation)
  - [Attributes](#Attributes%20-%20Most%20of%20these%20should%20probably%20not%20be%20changed)
  - [Methods](#Methods)

# MiroCode Constants
Removed the ones that aren&#39;t of use, but the full file can be found [here](http://labs.consequentialrobotics.com/miro-e/docs/viewfile.php?file=media/constants.py "here").

When using any of these in mirocode, put miro.constants. in front in order to be able to access them.

####  CAM_L, CAM_R
MiRo&#39;s cameras.  L - Left, R - Right.

####  CLIFF_L, CLIFF_R
MiRo&#39;s cliff sensors.

####  ILLUM_X
These constants refer to the LED lights on the back of MiRo.
X can be LF (Left Front), LM (Left Middle), LR (Left Rear), RF (Right Front), RM (Right Middle), RR (Right Rear), L (All Left), R (All Right), FRONT (Both Front), MID (Both Mid), REAR (Both Rear), or ALL (All Lights).

####  IMU_B, IMU_H
IMU sensors on MiRo&#39;s back and head.

####  JOINT_TILT, JOINT_LIFT, JOINT_YAW, JOINT_PITCH, JOINT_COUNT
JOINT_LIFT, JOINT_YAW, and JOINT_PITCH are all used for `neck_angle`. Not sure what the others are for but they were grouped together.

####  JOINT_DROOP, JOINT_WAG, JOINT_EYE_L/R, JOINT_EAR_L,/R
Joints that can be used in the `joint_position` method.

####  LIGHT_LF, LIGHT_RF, LIGHT_LR, LIGHT_RR
Constants that refer to MiRo&#39;s light sensors. LF - Left Front, RF - Right Front, LR - Left Rear, RR - Right Rear.

####  LIFT_RAD_MIN, LIFT_RAD_MAX
Minimum and maximum lift values (in radians). Minimum is 8deg, maximum is 60deg.

####  MIC_L, MIC_R, MIC_C, MIC_T
Referring to the sound sensors. Not sure what the ones labelled C and T refer to though.

####  PITCH_RAD_MIN, PITCH_RAD_MAX
Minimum and maximum pitch values (in radians). Minimum is -22deg, maximum is 8deg.

####  TILT_RAD_MIN, TILT_RAD_MAX
Minimum and maximum tilt values (in radians). Minimum and maximum are both -6deg.

####  WHEEL_DIAMETER_M
Returns the diameter of MiRo&#39;s wheels (0.090m).

#### WHEEL_TRACK_M
Returns the track length of MiRo&#39;s wheels (0.164m).

#### WHEEL_MAX_SPEED_M_PER_S
Max speed of MiRo&#39;s wheels (0.400m/s).

#### YAW_RAD_MIN, YAW_RAD_MAX
Minimum and maximum yaw values (in radians). Minimum is -55deg, maximum is 55deg.

# MirocodeInterface Class

## Instantiation
`robot = MirocodeInterface(pose_ctrl=True, cliff_reflex=True, robot_name=None, control_period=0.1)` - Always create your MirocodeInterface object at the start of any mirocode file.
`pose_ctrl` controls whether or not MiRo will use position correction
`cliff_reflex` controls whether or not you want the cliff reflex turned on - will automatically stop MiRo from moving forward when it detects a cliff.
`robot_name` - Sets a name for the robot - I think only needed if working with the actual robots, rather than MiRoCloud.
`control_period` - idk

## Attributes - Most of these should probably not be changed

#### active
True if MiRo is active, False if not.

#### clap_detector
Handles clap detection through the microphone.

#### cmd_vel, cmd_lin_vel, cmd_ang_vel
&#39;Construct objects for velocity corrections&#39;, according to the source code.

#### count
Doesn't seem to be used anywhere else in the MirocodeInterface object, but is incremented by one at the start of the `calculate_shake` method

#### current_clap
A list of the current clap state, as measured by each mic.

#### interface
Set by default to miro.lib.RobotInterface, with specific arguments. How the Mirocode interacts with the robot itself.

#### mirocode_thread_active
Boolean that represents whether the control thread is active.

#### new_accel_data
Stores most recent acceleration data registered by IMUs. Two lists (one for each IMU), each containing 3 lists of values (one for each dimension).

#### old_accel_data
Stores previous acceleration data registered by IMUs. Used for comparisons in some methods.

#### pub_annotation
Rospy publisher object used for annotating images in MiRoCode

#### sensor_period
Time that MiRo waits for new data to arrive.

#### sensors_updated
True/False depending on whether sensors have recently recieved new data from `sensors_callback`. Set to True in `sensors_callback`, and then set cleared again once that new data has been registered as recieved. 

#### shake, shake_calc_in_prog, shake_smooth, shake_smooth_i
Used in shake calculations - calculating magnitude of shake, after it has been smoothed and denoised.

#### shake_detected
A list of two booleans, that represent whether a shake has been detected by either IMU.

#### shake_thresh
The threshold for detecting a shake (not sure on units).

#### thread
Thread used by MirocodeInterface to interact with the website.

#### time_at_touch, time_at_body_touch, time_at_head_touch
Stores the time at which the most recent touch was detected by each sensor.

#### time_at_shake
A list of the most recent times a shake has been detected by each IMU

#### time_limit
Current time limit, expressed as the time at which the timer will finish (current time + duration of timer).

#### timer_watchdog_n, timer_watchdog_t
Used in conjunction with one another to prevent timeouts when waiting for time limit.

#### vision
Set by default to VisionInterface - how the MirocodeInterface object interacts with MiRo&#39;s vision capablities.

#### wag_freq
Frequency at which MiRo&#39;s tail is wagging - defaults to 0.

## Methods

#### calculate_shake(self)


#### camera_l_callback(self, frame)


#### camera_r_callback(self, frame)


#### configure_clap_detector(self, thresh)
Changes threshold of clap detector (threshold presumably in decibels).

#### configure_shake_detector(self, thresh=10)
Changes threshold of shake detector, and resets previous shake detections (not sure on units).

#### control_led(self, led, rgb, brightness)
Illuminates an LED on MiRo&#39;s body.
`led` attribute is a miro constant, e.g. `miro.constants.ILLUM_LF` for left front).
`rgb` is a list of length three containing the rgb code for the desired colour.
`brightness` is a 0-255 scale.

#### detect_clap(self, mic=None)
Returns true/false, depending on whether or not a clap has been detected.
If no mic has been passed, defaults to detecting through any of them.

#### detect_shake(self, imu=None)
Same function as detect_clap, but uses MiRo&#39;s IMUs (Internal Measurement Unit) to measure whether MiRo has been shaken. Run once before a while loop to clear previous shakes.
Again, if no IMU has been specified, defaults to detecting through any of them.

#### disconnect(self)
Disconnects from the robot, preventing it from doing anything else.

#### drive_distance(self, m, m_sec=0.3)
Drives MiRo a distance (in meters, `m`).
Optional argument `m_sec` controls MiRo&#39;s speed

#### find_ball(self, rgb, cam=None, prop=None, colour_range=15, min_rad=0.0078125)
Searches for a ball of the specified rgb colour through MiRo&#39;s cameras (can be specified, or can be left as None to search through all of them).
`rgb` - Colour of ball you&#39;re looking for
`cam` - Camera to search through (leave as None to search for a ball in each camera)
`prop` - What you want to be returned by the method, e.g.:
- MirocodeInterface.vision.loc_x for centre x-coordinate of ball
- MirocodeInterface.vision.loc_y for centre y-coordinate of ball
- MirocodeInterface.vision.loc for centre coordinates of ball
- MirocodeInterface.vision.rad for radius of ball
- MirocodeInterface.vision.cam for camera that the ball was detected through

if `prop` is left as None, then the method will return all details about the ball, in a tuple/list (one of the two)
`colour_range` - Colour threshold used when searching - not sure on specifics.
`min_rad` - Minimum radius a ball must have to be detected (presumably in metres)

#### find_colour(self, rgb, cam=None, prop=None, colour_range=15)
Searches for a specified colour through the cameras. Most arguments have the same purpose/can be the same things as in `find_ball`. The only difference is that instead of radius, we have `MirocodeInterface.vision.area`, that returns the proportion/area of pixels that are of the specified colour. Again, if `prop` is left as None, then the method will return all details about the colour, in a tuple/list (one of the two)

#### find_colour_at_centre(self, size, cam)
Returns average colour at specified centre square of a camera.
`size` is an int - 1 for centre 1 pixel, 9 for centre 9 etc etc. Use 0 for average of all camera pixels.
`cam` needs to be passed this time - use mirocode constants e.g. `miro.constants.CAM_L`

#### find_mirocube(self, cam=None, prop=None)
Searches for a mirocube through the specified camera (if camera is left as None, searches through all of them).
`prop` - again, what you want to be returned. The only new one is MirocodeInterface.vision.id , which would make the method return the ID of the Mirocube that has been detected - all others are the same as the ones from `find_ball`.
Again, if `prop` is left as None, then the method will return all details about the cube, in a tuple/list (one of the two)

#### find_shape(self, rgb, verts, cam=None, prop=None, colour_range=30, min_size=200)
Finds a shape of the specified rgb colour through the specified camera (if camera is left as None, searches through all of them).
`verts` - Number of vertices (corners) that the shape has. Must be between 3 and 8 (inclusive).
`prop` - What to return. Has identical possibilities to `find_colour`. Again, if `prop` is left as None, then the method will return all details about the shape, in a tuple/list (one of the two).

#### is_active(self)
Returns True if the robot is active, and False if not

#### joint_position(self, joint, pos)
Controls MiRo&#39;s joints, other than its neck.
Options for `joint` (all start with miro.constants.) - options for `pos`
`JOINT_EAR_L/R`  - 0.0 (Ear Facing Outwards), 1.0 (Ear Facing Forwards)
`JOINT_EYE_L/R` - maybe for eye (presumably)
`JOINT_DROOP` - 0.0 (Tail Up), 1.0 (Tail Down)
`JOINT_WAG` - 0.0 (Tail Left), 0.5 (Tail Straight), 1.0 (Tail Right)


#### microphones_callback(self, microphones)


#### mirocode_update(self, t, phase, wag_pos, channel, e)


#### neck_angle(self, joint, deg)
Controls where MiRo is looking. 
For `joint`, choose between yaw (left/right), pitch (up/down), and lift(raise/lower) (use the corrosponding mirocode constants to select a joint).
For `deg`, enter a value in degrees (positive values are looking down or left, negative ones are up or right).

#### play_tone(self, hz, sec, vol)
Plays a tone throught MiRo&#39;s speakers. 
`hz` - Frequency.
`sec` - Time to play the tone for.
`vol` - Volume to play the tone at.

#### read_body_acceleration(self)
Returns a list of three values - presumable the acceleration forces experienced by the IMU. I think the order is [Forwards, sideways, vertical], but I could be wrong

#### read_body_touch_sensor_list(self)
Returns a list of booleans, one for each body touch sensor. True means that a touch is currently being detected by the touch sensor represented by that position in the list, and False means that it has not.

#### read_cliff_sensor(self, pos, thresh=0.7)
Returns true or false depending on whether MiRo detects that a cliff is closer than the set threshold.
`pos` needs to be a miro constant - one of the two cliff sensors

#### read_cliff_sensor_list(self, thresh=0.7)
Functions the same as `read_cliff_sensor`, but instead of returning one value, it returns a list containing two booleans - the first one for the left cliff sensor, and the second for the right. No need to pass a `pos` argument.

#### read_head_acceleration(self)
Does the same as `read_body_acceleration`, but using readings from the IMU in MiRo&#39;s head

#### read_head_touch_sensor_list(self)
Returns a list of booleans, one for each head touch sensor. True means that a touch is currently being detected by the touch sensor represented by that position in the list, and False means that it has not.

#### read_light_sensor(self, pos)
Returns a decimal value for light level detected in the specified light sensor. `pos` is again a miro constant that dictates which sensor to use.

#### read_light_sensor_list(self)
Functions the same as `read_light_sensor` but returns a list of four values, one for each light sensor.

#### read_sonar_range(self)
Returns the sonar range - how far away the object directly in front of MiRo is (usually the surface it is on). Will return 1.0 if the distance is too great to measure

#### ready(self)
Used for periodic control loop: `while robot.ready():`

#### sensors_callback(self, sensors)
Recieves data from MiRo&#39;s sensors, and updates attributes. Also sets sensors_updated flag to True.

#### set_time_limit(self, dur)
For use with [wait_for_time_limit(self)](#wait_for_time_limitself)
Sets the time (in seconds, `dur`) that you want MiRo to wait for.

#### sleep(self, dur)
Wait for length of time specified as `dur` (in seconds) before doing anything else

#### speed(self, m_sec)
Sets MiRo&#39;s speed to a value in m/s (`m_sec`). Use positive values for forwards motion, and negative values for backwards.

#### target_mdk_release(self)


#### target_supports(self, feature)


#### term(self)
Terminates MiRo (sets active to false)

#### time_since_clap(self, mic=None)
How long (in seconds) since last clap was detected through the given mic. If `mic` is left as None, then how many seconds since last clap detected through any mic. `mic` can be set to any mir constant that refers to a mic on MiRo&#39;s body.

#### time_since_shake(self, imu=None)
How long (in seconds) since last shake was detected through the given IMU. If `imu` is left as None, then how many seconds since last shake detected through any IMU. `imu` can be set to miro.constants.IMU_B (body), miro.constants.IMU_H (head), or left as None, which defaults to both (returns time since either detected a shake.)

#### time_since_touch(self, pos=&#34;any&#34;;)
How long (in seconds) since last touch was detected through the given part. If `pos` is left as any, then how many seconds since last touch detected anywhere. `pos` can be set to &#34;body&#34;, &#34;head&#34;, or &#34;any&#34;.

#### turn_angle(self, deg, deg_sec=30)
Turn MiRo a certain angle.
`deg` is the angle to turn through. Again, use positive values for left turns, and negative values for right.
`deg_sec` is the speed to turn. This time round use only positive values, as it is non-directional.

#### turn_speed(self, deg_sec)
Sets MiRo&#39;s rotational speed to a value in degrees per second (`deg_sec`). Use positive values for leftwards (anticlockwise) rotation, and negative values for rightwards (clockwise).

#### wag_frequency(self, hz)
Controls the speed of MiRo&#39;s tail.
`hz` - How many times per second MiRo&#39;s tail will wag.

#### wait_for_time_limit(self)
For use with [set_time_limit(self, dur)](#set_time_limitself-dur)
Returns a boolean depending on whether or not the time limit set in `set_time_limit` has been exceeded yet (True if not exceeded, False if exceeded).
