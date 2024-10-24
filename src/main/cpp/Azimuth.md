
# Azimuth Wheel Dynamics

We need to know the theoretical maximum acceleration and velocity that the azimuth wheel can spin in order to identify the motion profile used to control the wheel.

For the sake of this discussion, the following constants are given:
* Gear Ratio: 13.37 motor rotations per mechanism rotation
* Rotations to turn: 0.25 mechanism rotations
* Rotations to turn: 3.3425 motor rotations (mechanism rotations multiplied by gear ratio)

## Azimuth Wheel Dynamics - Angular Acceleration (mechanism space)

To find angular acceleration, we can use the torque formula of:
$$ \tau = I \alpha $$

where:
* T - Torque in Nm
* I - Moment of Inertia in kg m^2
* a - Angular acceleration in RPS^2

### Torque

Torque is provided to us by the motor manufacturer. Specifically, we can use the motor curve graph provided to identify the stall torque, or the maximum torque the motor can provide. The graph is below:

![https://motors.vex.com/media/wysiwyg/MotorCurve-8V_4.PNG](https://motors.vex.com/media/wysiwyg/MotorCurve-8V_4.PNG)

The stall torque is identified as 2.7 Nm

### Moment of Inertia

Moment of Inertia of the wheel can be referenced in the image below, with a object type of `Hoop about diameter`:

![https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQGWcFomf1A8_v9kNHSoJTDwGpSsS82svv_evmhWReI7w&s](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcQGWcFomf1A8_v9kNHSoJTDwGpSsS82svv_evmhWReI7w&s)

Therefore the final calculations for Moment of Inertia become:

$$ I = {1\over2}M R^2 $$
$$ I = 0.0005858 kg * m^2$$

where:
* M - Mass of the wheel (kg)
    * 0.227 kg [WCP Aluminum wheel](https://wcproducts.com/products/aluminum-wheels)
* R - Radius of the wheel (m)
    * 0.0508 m [WCP Aluminum wheel](https://wcproducts.com/products/aluminum-wheels)

### Final Calculation

$$ \tau = I \alpha $$
$$ \alpha = {\tau \over I} $$
$$ \alpha = {2.6 Nm \over 0.0005858 kg * m^2} $$
$$ \alpha = 8876.669 {rad \over sec^2} $$

## Azimuth Wheel Dynamics - Angular Acceleration (motor space)

Now that we have the angular acceleration in the mechanism space, we need to obtain the angular acceleration in motor space with the correct units. The unit conversion requires us to get from RPS^2 to RPM^2.

$$ {8876.669 rad \over sec^2} {| \over |} {1 rot_{mech} \over 2 \pi rad} {| \over |} {13.37 rot_{motor} \over 1 rot_{mech}} {| \over |} {3600 sec^2 \over 1 min^2}$$ 

We then end up with an angular acceleration in the motor space of:

$$ 67,999,246.69{rotations \over min^2} $$

## Azimuth Wheel Motion Profile

Now that we know the max angular acceleration, we can identify the motion profile that the wheel should follow when attempting to drive to a position. 

### Distance to max velocity

To create the motion profile, we need to identify how far the wheel has rotated before hitting max velocity when accelerating at max acceleration.

The following formula is used to identify position from acceleration and max velocity:

$$ x = 0.5 a t^2 $$
$$ \Delta t = {\Delta V \over a} $$
$$ x = 0.5 a ({\Delta V \over a})^2 $$
$$ x = 0.5 a {\Delta V^2 \over a^2} $$
$$ x =  {\Delta V^2 \over 2a} $$

where
* V = Final Velocity (RPM)
  * 5676 RPM for a Neo 1.1
* a = Angular Acceleration (RPM^2)
  * 67,999,246.69 RPM^2 (calculated earlier)

Solving yields a distance of 0.2369 rotations of the motor to reach max acceleration

### Motion Profile

We now have all the information we need for the profile. We ramp at max acceleration until a set distance, then travel at max velocity, and then decelerate again. The table below showcases the numbers:

| Distance | Speed | Notes |
|---|---|---|
| 0 | 0 | Start |
| 0.2368 rotations | 5676 RPM | Ramping at max acceleration |
| 3.1056 rotations | 5676 RPM | Constant speed at 5676 RPM |
| 3.425 rotations | 0 | Decelerate at max acceleration to hit the desired rotations |

## Testing

Depending on real world acceleration numbers, we may not have a trapezoid for the motion profile. In that case, we get a triangle and we can solve in an easier manner.

The following formula is used to identify the final velocity given an initial velocity and an acceleration:

$$ V_f^2 = V_i^2 + 2 a \Delta x $$

where:
* Vf = Final Velocity (RPM)
* Vi = Initial Velocity (RPM)
* a = Angular Acceleration (RPM^2)
* X = Distance traveled (m)

Solving for Vf would yield:

$$ V_f = \sqrt { V_i^2 + 2 a \Delta x } $$
$$ V_f = \sqrt { 2 a \Delta x } $$

Once we have a final velocity, we can divide that by the max velocity of the motor which gives us the duty cycle the motor will be running at at final velocity

$$ {Duty}_{cycle} = {\sqrt { 2 a \Delta x } \over 5676 } $$ 

If we run the motor at 12V, then we can determine the voltage at final velocity (or the peak of the profile)

$$ Volt = {12 \sqrt { 2 a \Delta x } \over 5676 } $$ 

If we have a delta x of 3.3425 rotations of the motor, the peak of the triangle (aka. halfway to the target) would be our desired max speed before we have decelerate again.

Plugging that in, we get a final equation of:

$$ Volt = {12 \sqrt { 1.671 a} \over 5676 } $$ 

Some possible a values and the peak voltage for reference:

| a | Time to max speed | Voltage |
| --- | --- | --- |
| 56760 | 0.1 seconds | 0.92 V |
| 567600 | 0.01 seconds | 2.91 V |
| 5676000 | 0.001 seconds | 9.2 V |