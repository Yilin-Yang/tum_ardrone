tum_ardrone
================================================================================

This package is a fork of [`Eurecat/tum_ardrone`](https://github.com/Eurecat/tum_ardrone),
which is itself a fork of [`tum-vision/tum_ardrone`](https://github.com/tum-vision/tum_ardrone)
that ports its functionality from ROS Indigo Igloo to ROS Kinetic Kame.

The purpose of _this_ repository is to:
1. Update this repositories instructions and references to be more accessible to laypeople,
2. Refactor, clean, and document the original `tum_ardrone` code to be more
   modular and more easily maintainable,
3. Extend the existing functionality of `tum_ardrone`

The original authors of this code were (from a casual perusal of the commit
history and some repo errata) Jakob Engel and Stefan Wilkes at the Technical
University of Munich. You can find more information by reading the papers
linked below, or by reading this package's page [on the ROS Wiki.](http://wiki.ros.org/tum_ardrone)

---

This package contains the implementation corresponding to the following publications:

- [Scale-Aware Navigation of a Low-Cost Quadrocopter with a Monocular Camera](https://vision.in.tum.de/_media/spezial/bib/engel14ras.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Camera-Based Navigation of a Low-Cost Quadrocopter](https://vision.in.tum.de/_media/spezial/bib/engel12iros.pdf) (J. Engel, J. Sturm, D. Cremers)
- [Accurate Figure Flying with a Quadrocopter Using Onboard Visual and Inertial Sensing](https://vision.in.tum.de/_media/spezial/bib/engel12vicomor.pdf) (J. Engel, J. Sturm, D. Cremers)

You can find a [video on YouTube.](https://www.youtube.com/watch?feature=player_embedded&v=eznMokFQmpc)

This package builds on the well known monocular SLAM framework PTAM presented
by Klein & Murray in their paper at ISMAR07. Please study the original PTAM
website and the corresponding paper for more information on this part of the
software. Also, be aware of the license that comes with it.

While this code works for both the AR.Drone 1.0 and 2.0, the default-parameters
are optimized for the AR.Drone 2.0.

**Note: the instructions below are being revised and may not be accurate as of
this time.**

Installation
--------------------------------------------------------------------------------

These instructions assume the use of **ROS Kinetic Kame** on a system running
**Ubuntu 16.04.** Newer versions of Ubuntu _should_ work fine, but newer (or
older) versions of ROS likely won't work.

### Build Dependencies

``` bash
# Install ROS
sudo apt install ros-kinetic-desktop-full

# Install build dependencies
sudo apt install ros-kinetic-joystick-drivers \
                 daemontools \
                 libudev-dev \
                 libiw-dev \
                 graphviz \
                 doxygen

# Install dependent ROS packages
sudo apt install ros-kinetic-ardrone-autonomy

# probably overkill, but shouldn't hurt
sudo apt install ros-kinetic-hector-*
```

### Build with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/tum-vision/tum_ardrone.git -b kinetic-devel
cd ..
catkin init
catkin config --extend /opt/ros/kinetic # or whatever your install path was
catkin build -v tum_ardrone # build with verbose output
```

### Build Directly with cmake
Note that this still requires catkin, i.e. you still need to `cd ~/catkin_ws/ &&
source devel/setup.sh` in order for this to work properly.

``` bash
cd /path/to/project/root
mkdir build && cd build
cmake ..
make -j
```

**This isn't recommended** as a means of building this package for actual use.
It's more of a means to more easily check for compilation errors, or to
generate [doxygen output](#Doxygen) or a [compilation database](https://clang.llvm.org/docs/JSONCompilationDatabase.html)
in a more convenient spot.

Quick Start
--------------------------------------------------------------------------------

#### Launch the Nodes

``` bash
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone.launch
```

#### Check Status

On the GUI, under Drone Communication Status, you should see:
- Drone Navdata: XHz (X > 100)
- Pose Estimates: 33Hz

#### Keyboard Control

- focus `drone_gui` window
- press `<ESC>` to activate KB control
- fly around with keyboard (see [`drone_gui`](#drone_gui) for key assignments)

#### Joystick Control

- `rosrun joy joy_node`
- press PS button on controller to activate it
- fly around (see [`drone_gui`](#drone_gui) for key assignments)

#### Autopilot

- Type the command `autoInit 500 800` into the top-left text-field.
- Click `Clear and Send` (maybe click `Reset` first).
  - `Clear` clears the command queue.
  - `Send` sends your commands.
  - `Reset` resets the state estimator.
  - The drone will take off and initialize PTAM, then hold position.
- Click on the video to interactively set target (relative to current position);
  see [`drone_stateestimation`](#drone_stateestimation) for more details.
  - First, fly up 1m and then down 1m to facilitate a good scale estimate.
  - Don't get too ambitious, e.g. by starting with a horizontal flight over
    uneven terrain.
- Always have a finger on ESC or on the joystick for emergency-keyboard control :)

Nodes
--------------------------------------------------------------------------------

### `drone_stateestimation`

Estimates the drone's position based on sent navdata, sent control commands, and PTAM.

*This requires messages to be sent on both `/ardrone/navdata (>100Hz)` and
`/ardrone/image_raw (>10Hz)`,* i.e. a connected drone with a running
`ardrone_autonomy` node, or a `.bag` replay of at least those two channels.
`ardrone_autonomy` should be started with:

``` bash
rosrun ardrone_autonomy ardrone_driver _realtime_navdata:=True _realtime_video:=True _navdata_demo:=0
```


#### Subscribed Topics


- `/ardrone/navdata`
- `/ardrone/image_raw`
- `/cmd_vel`
- `/tum_ardrone/com`


#### Published Topics

- `/ardrone/predictedPose`
- `/tum_ardrone/com`


#### Services

None.


#### Parameters


- `~publishFreq`: The frequency at which the drone's estimated position is
  calculated & published. Default: 30Hz
- `~calibFile`: The camera calibration file. If not set, the defaults are used
  (`camcalib/ardroneX_default.txt`).
- `UseControlGains`: Whether to use control gains for EKF prediction.
- `UsePTAM`: Whether to use PTAM pose estimates as EKF update
- `UseNavdata`: Whether to use Navdata information for EKF update
  - UsePTAM and UseNavdata are set to false, the EKF is never updated and acts as a pure simulator, prediciting the pose based on the control commands received (on /cmd_vel). Nice for experimenting.

- `PTAMMapLock`: Whether to lock the PTAM map (no more keyframes).
- `PTAMSyncLock`: Whether to lock PTAM map sync (fix scale and pose offsets, etc.)
- `PTAMMaxKF`: The maximum number of keyframes taken by PTAM.

- `PTAMMinKFDist`: The minimum distance between two keyframes (in meters).
- `PTAMMinKFWiggleDist`: The minimum distance between two keyframes (relative to
  mean scene depth).
- `PTAMMinKFTimeDiff`: The minimum time between two keyframes.
  - PTAM takes a new keyframes if (`PTAMMinKFTimeDiff` AND (`PTAMMinKFDist` OR
  `PTAMMinKFWiggleDist`)), and tracking is good, etc.

- `RescaleFixOrigin`: If the scale of the Map is reestimated, only one point in
  the mapping PTAM <-> World remains fixed.
  - If `RescaleFixOrigin == false`, this is the current pos. of the drone (to
    avoid sudden, large "jumps"). this however makes the map "drift".
  - If `RescaleFixOrigin == true` (default), this is the initialization point
    where the second keyframe has been taken (drone pos. may jump suddenly, but
    map remains fixed.). The fixpoint may be set by the command `lockScaleFP`.

- `c1 ... c8`: prediction model parameters of the EKF. see "Camera-Based
  Navigation of a Low-Cost Quadrocopter"


#### Required tf transforms


TODO


#### Provided tf transforms


TODO


#### Using State Estimation

To properly estimate PTAM's scale, it is best to fly up and down a little bit
(e.g. 1m up and 1m down) immediately after initialization.

The estimator spawns two GUI windows: one showing the video and PTAM's map
points, the other showing the map. To issue commands (movement waypoints, or
commands to PTAM), focus the respective window and hit a key. This generates
a command on `/tum_ardrone/com`. See below for details.


###### Video Window

![Video window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=video.png)

| Key   | `/tum_ardrone/com` message | Action  |
|-------|----------------------------|---------|
| r     | "p reset"                  | resets PTAM |
| u     | "p toggleUI"               | changes view |
| space | "p space"                  | takes first / second Keyframe for PTAM's initialization |
| k     | "p keyframe"               | forces PTAM to take a keyframe |
| l     | "toggleLog"                | starts / stops extensive logging of all kinds of values to a file |
| m     | "p toggleLockMap"          | locks map, equivalent to parameter PTAMMapLock |
| n     | "p toggleLockSync"         | locks sync, equivalent to parameter PTAMSyncLock |

Clicking on the video window will generate waypoints, which are sent to
`drone_autopilot` (if running):

- left-click: fly `(x, y, 0)` meters relative to the drone's current position.
  The image-center represents `(0, 0)` (i.e. the drone's current position), and
  the borders represent displacements of 2 meters from that point.
- right-click: fly `(0, 0, y)` meters and rotate yaw by `x` degrees.
  The image-center is `(0, 0)`, while the borders represent displacements of
  2 meters and 90 degrees, respectively.

Note that these waypoints are given in the coordinate frame of the drone's
_map_ and _not_ in the drone's local coordinate frame. This can be confusing
when piloting the drone through the video window.

For instance, if the drone takes off facing away from you (such that the
x-axis of its "world map" points away from you), then yaws 180 degrees (so that
its _local_ x-axis faces towards you), then sending the waypoint `(1.5, 0, 0)`
will tell the drone to fly "backwards" towards its map's `+X`, _not_ "forwards"
(from the perspective of its camera) towards you.


###### Map Window

![Map window](http://wiki.ros.org/tum_ardrone/drone_stateestimation?action=AttachFile&do=get&target=map.png)

| Key   | `/tum_ardrone/com` message | Action  |
|-------|----------------------------|---------|
| r     | "f reset"                  | resets EKF and PTAM |
| u     | "m toggleUI"               | changes view |
| v     | "m resetView"              | resets viewpoint of viewer |
| l     | "toggleLog"                | starts / stops extensive logging of all kinds of values to a file |
| v     | "m clearTrail"             | clears green drone-trail |

### `drone_autopilot`

PID controller for the drone. Also includes basic way-point-following and
automatic initialization. Requires [drone_stateestimation](#drone_stateestimation)
to be running. The target is set via the `/tum_ardrone/com` topic.


#### Subscribed Topics


- /ardrone/predictedPose


#### Published Topics

- `/cmd_vel`
- `/ardrone/land`
- `/ardrone/takeoff`
- `/ardrone/reset`


#### Services

None.


#### Parameters


- `~minPublishFreq`: usually, a control command is sent for each pose estimate
  received from `drone_stateestimation.` However, if no pose estimate is
  received for more than `minPublishFreq` milliseconds, a HOVER command is sent,
  causing the drone to hover if for example `drone_stateestimation` is shut down
  suddenly. Default: 110.
- `Ki_X, Kd_X, Kp_X`: PID controller parameters for roll/pitch, gaz (up/down) and yaw.
- `max_X`: maximal respective control command sent (ever).
- `rise_fac`: rise commands are larger than respective drop commands by this
  factor. This is due to the drone sometimes dropping unpredictably fast for
  large drop commands, however rising sometimes requires large rise commands.
- `aggressiveness`: multiplied to PI-component of all commands sent. Low values
  lead to the drone flying "slower".


#### Required tf transforms


TODO


#### Provided tf transforms


TODO


#### Using the Autopilot

The behavior of the autopilot is set by sending commands on `/tum_ardrone/com`
of the form `c COMMAND`. A queue of commands is kept. As soon as one
command is finished (for example, when a waypoint has been reached), the
current command is popped from the queue, and the next command is executed. The
queue can be cleared by sending `c clearCommands`. Commands can be sent using
the [`drone_gui`](#drone_gui) node. Some example scripts can be found in
`/flightPlans/*.txt`. Possible commands are:

- `autoInit [int moveTimeMS] [int waitTimeMS] [int riseTimeMs] [float initSpeed]`

        takes control, starts drone, initializes PTAM. That is:
        - `starts the drone & and waits riseTimeMs, the drone will rise to approx.`
          a height of 1m.
        - `initializes PTAM by taking the first KF, flying up (sending initSpeed as control command) for moveTimeMS,`
          waiting waitTimeMS and then taking the second KF.
          This is done until success (flying up and down respectively).
        - `good default values are "autoInit 500 800 4000 0.5"`

- `autoTakeover [int moveTimeMS] [int waitTimeMS] [int riseTimeMs] [float initSpeed]`

        takes control, initializes PTAM. The same as autoInit ...,
        but to be used when the drone is already flying (skipps the first step).

- `takeoff`

        - `takes control, starts drone.`
        - `does not reset map or initialize PTAM`

- `setReference [doube x] [double y] [double z] [double yaw]`

        sets reference point (used in goto X X X X).

- `setMaxControl [double cap = 1.0]`

        maximal control sent is capped at [cap], causing the drone to fly slower.

- `setInitialReachDist [double dist = 0.2]`

        drone has to come this close to a way point initially

- `setStayWithinDist [double dist = 0.5]`

        drone has to stay this close to a way point for a certain amount of time.

- `setStayTime [double seconds = 2.0]`

        time the drone has to stay within [stayWithinDist] for target to be reached.

- `clearCommands`

        clears all targets, such that the next command is executed immediately.

- `goto [doube x] [double y] [double z] [double yaw]`

        flies to position (x,y,z yaw), relative to current reference point.
        blocks until target is reached according to set parameters

- `moveBy [doube x] [double y] [double z] [double yaw]`

	moves by (x,y,z,yaw), relative to the current target position
        blocks until target is reached according to set parameters

- `moveByRel [doube x] [double y] [double z] [double yaw]`

	moves by (x,y,z,yaw), relative to the current estimated position
        of the drone
        blocks until target is reached according to set parameters

- `land`

	initializes landing (use auto-land of drone)

- `lockScaleFP`

	sets the one point that does not change when the scale is re-estimated
        to the current drone position. The scaleFP can only be set when PTAM is
        good, i.e. this is delayed until PTAM is initialized and good.
        Need to set useWorldFixpoint in dynammic_reconfigure.


### `drone_gui`

This node offers a simple QT GUI to control the [`drone_autopilot`](#drone_autopilot)
node, the [`drone_stateestimation`](#drone_stateestimation)
node and fly the drone manually via keyboard or joystick.


#### Subscribed Topics


- `/cmd_vel`
- `/tum_ardrone/com`
- `/ardrone/takeoff`
- `/ardrone/land`
- `/ardrone/reset`
- `/ardrone/predictedPose`
- `/ardrone/navdata`
- `/joy`


#### Published Topics

- `/cmd_vel`
- `/tum_ardrone/com`
- `/ardrone/takeoff`
- `/ardrone/land`
- `/ardrone/reset`


#### Services

- calls `/ardrone/togglecam`
- calls `/ardrone/flattrim`


#### Parameters


None.


#### Required tf transforms


None.


#### Provided tf transforms


None.


#### Using the Drone GUI

![Drone GUI](http://wiki.ros.org/tum_ardrone/drone_gui?action=AttachFile&do=get&target=ui.png)

###### Monitor Drone, Autopilot and State Estimation Nodes (top-right part).

On the top-right, the current publish-frequency of important topics is displayed:
- Drone Navdata: monitors `/ardrone/navdata`. Should be around 150 to 200Hz with
  a connected drone, and running (and correctly configured) `ardrone_autonomy`
  node.
- Drone Control: monitors `/cmd_vel`. This is the frequency of how often control
  commands are published (and sent to the drone).
- Pose Estimates: monitors `/ardrone/predictedPose`. These are the state
  predictions generated by the `drone_stateestimation` node. By default, this
  should be 30Hz.
- Joy Input: monitors /joy. If you have a connected game-pad and running
  joy_node, this should be different from 0Hz.
- Pings (RTT): current wireless LAN latency (RTT in ms) for 500B and 20kB
  packages. If these are too high, reduce WLAN clutter.


###### Manual or Joystick Control

The current control source has to be set (i.e. joystick or KB). The autopilot is
only allowed to send control commands, if this is set to autopilot.

- Joystick control requires a connected joystick and running `rosrun joy joy_node`. We use a PS3 SixAxis controller.
  to make the controller work, a small linux-hack is required (set controller rights).
  - left joystick is horizontal pos. control; right joystick is height and yaw control.
  - L1 to take off, release L1 to land.
  - R1 to toggle emergency state.
  - By moving any of the two joysticks, the Control Source is immediately sent
    to Joystick. This can be used for safety, e.g. the autopilot does something
    dangerous -> immediately take over, disabling the autopilot and enabeling
    manual control).

- Keyboard control requires that the GUI window be focused, but NOT the
  upper-left text field.
  - Make the GUI the active window and press ESC to immediately enable keyboard
    control.
  - This can be used for safety, e.g. the autopilot does something dangerous ->
    press ESC and immediately take over, disabling the autopilot and enabling
    manual control.
  - q,a: fly up & down.
  - i,j,k,l: fly horizontally.
  - u,o: rotate yaw.
  - F1: toggle emergency
  - s: takeoff
  - d: land

- Buttons `Land`, `Takeoff`, `ToggleCam`, `Emergency` (toggles emergency state).


###### Autopilot Control

- Write commands in top-left text field (or load example from one of the
  files). You can simply add `.txt` files to `flightplans/`.
- Click Send to transmit commands to the autopilot's command queue
  (automatically sets Control Source to Autopilot).
- Click Clear to clear autopilot command queue and current target.
- Click Reset to reset Autopilot, PTAM and EKF.


Building Doxygen
--------------------------------------------------------------------------------
This project's `CMakeLists.txt` files are configured to generate [doxygen](http://www.stack.nl/~dimitri/doxygen/)
output. As of time of writing, this package's codebase lacks actual doxygen
annotations; however, the doxygen output will still contain UML Inheritance
Diagrams, collaboration diagrams, and so on.


Troubleshooting
--------------------------------------------------------------------------------

- If drone doesn't start:
  - Is the battery empty? (The AR.Drone will not start if its charge is lower than ~18%.)
  - Is the drone in emergency state? (If so, the four LED's on the drone's
    landing gear are red. click Emergency to reset).
- Cannot control drone:
  - Have you selected the correct control source?
- Drone flight is unstable using the autopilot:
  - Adjust / Reduce control parameters using `dynamic_reconfigure` (see `readme_autopilot`).
- The drone is broken:
  - Buy a new one. [Seriously, they're pretty cheap, these days.](https://www.amazon.com/Parrot-AR-Drone-2-0-Elite-Quadcopter/dp/B00FS7SU7K/)


Known Bugs & Issues
--------------------------------------------------------------------------------

- As the software was originally developed for the AR.Drone 1.0, the pressure
  sensor and magnetometer are not used.
- `drone_stateestimation` crashes occasionally when PTAM initialization fails.
  - This tends to happen if there is no baseline in between the first two
  keyframes, which hardly ever happens in practice. The crash occurs in the PTAM
  code.


Tips and Tricks
--------------------------------------------------------------------------------

#### Camera Calibration

- front, old drone: `0.672049, 0.899033, 0.502065, 0.513876, 0.271972`
- front, new drone: `0.771557, 1.368560, 0.552779, 0.444056, 1.156010`

Calibrate with `ethzasl_ptam`.
To work with colored images, make the following changes in `src/CameraCalibrator.cc`:
- Add `#include <cv_bridge/cv_bridge.h>`
- Change the function named `imageCallback(...)` to:
```cpp
void CameraCalibrator::imageCallback(const sensor_msgs::ImageConstPtr & img)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	if(mCurrentImage.size().x != img->width || mCurrentImage.size().y != img->height)
		mCurrentImage.resize(CVD::ImageRef(img->width, img->height));

	memcpy(mCurrentImage.data(),cv_ptr->image.data,img->width * img->height);

	mNewImage = true;
}
```


#### Parameters: `c1` to `c8`

These can be easily estimated by:
- Recording a flight: `rosbag record -O calibFlight.bag /ardrone/image_raw /ardrone/navdata /cmd_vel`
- Playing back that flight: `rosbag play -l calibFlight.bag`
- Starting two `stateestimation` nodes, one with remapped name and output:
   - `rosrun tum_ardrone drone_stateestimation __name:=drone_stateestimationn2 /ardrone/predictedPose:=/ardrone/predictedPose2`
   - `rosrun tum_ardrone drone_stateestimation`
- Plotting the respective estimated values
  - e.g.: `rxplot /ardrone/predictedPose/dx,/ardrone/predictedPose2/dx`
- Using dynamic reconfigure to:
  - In `drone_stateestimation2`, use only control gains
  - In `drone_stateestimation`, use NO control gains, but instead navdata / speeds / PTAM.
- Playing around with `c_i` in `drone_stateestimation2` such that the graphs
  roughly match each other.


#### Parameters: PID control

These can be approximated in "simulation" based on the values of `c1` to `c8`:
- Play back any record, to make `stateestimation` run (don't play `/cmd_vel`)
  - `rosbag play -l calibFlightZ.bag --topics /ardrone/image_raw /ardrone/navdata`
- Run `stateestimation`, in dynamic reconfigure, set only control updates to be used.
- Run `drone_gui`, autopilot, and control.
- Plot control / respective pose.
  - e.g. `rxplot /ardrone/predictedPose/yaw /cmd_vel/angular/z`


License
--------------------------------------------------------------------------------

The major part of this software package - that is everything except PTAM - is
licensed under the GNU General Public License Version 3 (GPLv3), see
[`http://www.gnu.org/licenses/gpl.html`](http://www.gnu.org/licenses/gpl.html).

PTAM (comprised of all files in `/src/stateestimation/PTAM`) has it's own
license that you can find here: [`http://www.robots.ox.ac.uk/~gk/PTAM/download.html`](http://www.robots.ox.ac.uk/~gk/PTAM/download.html).
This license, in particular, prohibits commercial use of this software.
