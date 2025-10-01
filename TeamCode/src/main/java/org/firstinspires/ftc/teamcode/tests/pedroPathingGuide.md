# 🚀 Getting Started with PedroPathing (2.0.0+)

PedroPathing is a path-planning and following library for FTC robots.  
This guide walks you through **setup → path creation → following paths → debugging**.

---

## 1. Setup the Follower

The **Follower** is the brain that tracks your robot’s position and follows paths.

```java
// Create follower using your Constants class
Follower follower = Constants.createFollower(hardwareMap);

/* *** Important */
/* Since we have different robots and different constants for each one run this instead */
// Initialize the robot hardware class
RobotHardware robot = new RobotHardware(hardwareMap);
// Make sure follower has the correct constants (Automatic)
follower = robot.setPedroConstants();


// Set your starting position on the field
follower.setStartingPose(new Pose(0, 0, 0)); // x=0, y=0, heading=0 radians
```

Call `follower.update();` in your OpMode loop to keep it running.

---

## 2. Create Paths

PedroPathing supports **lines** and **curves**.  
Each path segment connects Poses (x, y, heading).

### Straight Line Path
```java
Path line = new Path(
    new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0))
);
```
This moves from `(0,0)` to `(24,0)`.

### Curved Path
```java
Path curve = new Path(
    new BezierCurve(
        new Pose(24, 0, 0),     // start
        new Pose(36, 12, 0),    // control point
        new Pose(48, 0, 0)      // end
    )
);
```
This creates a smooth curve instead of a sharp turn.

---

## 3. Chain Paths Together

Instead of following one segment at a time, you can build a chain.

```java
PathChain chain = follower.pathBuilder()
    .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)))
    .addPath(new BezierCurve(new Pose(24, 0, 0),
                             new Pose(36, 12, 0),
                             new Pose(48, 0, 0)))
    .build();
```

---

## 4. Make the Robot Follow

Tell the follower to follow your path or chain:

```java
follower.followPath(chain);
```

In your loop:
```java
@Override
public void loop() {
    follower.update();
    follower.telemetryDebug(telemetry); // see pose + debug info
}
```

The robot will move along the path until done.

---

## 5. Detect When a Path is Done

You can check if the robot is finished:
```java
if (!follower.isBusy()) {
    // Path finished — do next action
}
```