# Panels Quick-Start Guide (FTC)

Panels is a real-time telemetry and configuration dashboard for FTC.  
This guide explains how to set it up and use it in your robot code.

---

## 1. Make Your Class Configurable

Add `@Configurable` at the top of your class.  
This tells Panels that it should look for public variables to display.

```java
import com.ftcontrol.config.Configurable;

@Configurable
public class MyRobot {
    // public variables will appear on Panels
    public static double motorPower = 0.5;
    public static int servoPosition = 0;
}
```

---

## 2. Exclude Public Variables from Panels

If a variable must remain public but should **not** appear on Panels, use `@IgnoreConfigurable`:

```java
import com.ftcontrol.config.IgnoreConfigurable;

@IgnoreConfigurable
public static double internalCounter = 0;
```

---

## 3. Create PanelsTelemetry

In your OpMode, initialize Panels telemetry:

```java
import com.ftcontrol.telemetry.PanelsTelemetry;

public class MyOpMode extends OpMode {
    private PanelsTelemetry panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = new PanelsTelemetry(hardwareMap.appContext);
    }
}
```

---

## 4. Add Data to PanelsTelemetry

Use `debug()` to send variables or values to Panels:

```java
@Override
public void loop() {
    // Add robot values
    panelsTelemetry.debug("Motor Power", MyRobot.motorPower);
    panelsTelemetry.debug("Servo Position", MyRobot.servoPosition);

    // Update Panels and also send to Driver Station telemetry
    panelsTelemetry.update(telemetry);
}
```

---

## Key Points

- **Only public variables** appear on Panels by default.
- `@Configurable` is required at the **class level**.
- `@IgnoreConfigurable` hides a public variable from Panels.
- Use `PanelsTelemetry.debug()` to display runtime data.
- Always call `panelsTelemetry.update(telemetry)` in your loop to refresh both Panels and DS telemetry.

---

This minimal setup is usually enough to **start tuning variables live** and debug your robot in real-time using Panels.
