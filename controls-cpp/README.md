# **Project 3: Cascade Control for Quadrotor in C++**
___
## **Task 1:** Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  
You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.
This file contains four sections:
1. Task 1: Provide a Writeup / README.
2. Task 2: . 
3. Task 3: .
___
## **Task 2:** Rebuilding Python Controller into C++

### **2.1:** Implement body rate control.

#### Function `GenerateMotorCommands()`

```
VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  float Fc = collThrustCmd; // [N]
  float Mx = momentCmd.x; // [Nm]
  float My = momentCmd.y; // [Nm]
  float Mz = momentCmd.z; // [Nm]

  // Solve:
  // [1  1  1  1][F1]   [ Fc ]       k_m (drag)            L
  // [1 -1 -1  1][F2] = [Mx/l], K = ------------ , l = ---------
  // [1  1 -1 -1][F3]   [My/l]       k_f (lift)         sqrt(2)
  // [1 -1  1 -1][F4]   [Mz/K]

  float l = L / sqrt(2); // [m], 45deg angle

  float f1 = (Fc + Mx/l + My/l + Mz/(-kappa)) / 4; // [N]
  float f2 = (Fc - Mx/l + My/l - Mz/(-kappa)) / 4; // [N]
  float f3 = (Fc - Mx/l - My/l + Mz/(-kappa)) / 4; // [N]
  float f4 = (Fc + Mx/l - My/l - Mz/(-kappa)) / 4; // [N]

  cmd.desiredThrustsN[0] = f1; // front left
  cmd.desiredThrustsN[1] = f2; // front right
  cmd.desiredThrustsN[2] = f4; // rear left << REALLY WEIRD
  cmd.desiredThrustsN[3] = f3; // rear right << REALLY WEIRD

  return cmd;
}
```


#### Function `BodyRateControl()`

```
V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  V3F momentCmd;
  
  V3F pqr_error = pqrCmd - pqr;

  momentCmd[0] = Ixx * kpPQR.x * pqr_error.x;
  momentCmd[1] = Iyy * kpPQR.y * pqr_error.y;
  momentCmd[2] = Izz * kpPQR.z * pqr_error.z;
  
  return momentCmd;
}
```

#### Tune parameter `Kp_pqr` 



### **2.2:** Implement body rate control.

#### Function `RollPitchControl()`

```
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  V3F bankCmd;
  V3F bankActual;

  float accelThrust = collThrustCmd / mass;

  if(collThrustCmd > 0){
      bankCmd.x =  accelCmd.x / accelThrust;
      bankCmd.y =  accelCmd.y / accelThrust;

      bankCmd.x = - CONSTRAIN(bankCmd.x, -maxTiltAngle, maxTiltAngle);
      bankCmd.y = - CONSTRAIN(bankCmd.y, -maxTiltAngle, maxTiltAngle);

      bankActual.x = R(0,2);
      bankActual.y = R(1,2);

      V3F bankError = bankCmd - bankActual;

      V3F bankCmdDot = kpBank * bankError;

      pqrCmd.x = (R(1,0) * bankCmdDot.x - R(0,0) * bankCmdDot.y) / R(2,2);
      pqrCmd.y = (R(1,1) * bankCmdDot.x - R(0,1) * bankCmdDot.y) / R(2,2);

  }
  else {
      pqrCmd.x = 0;
      pqrCmd.y = 0;
  }
  return pqrCmd;
}
```

#### Tune parameter `Kp_bank`



### **2.3:** Position/velocity and yaw angle control

#### Function `LateralPositionControl()`

```
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
  accelCmd.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  V3F posError = posCmd - pos;

  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);

  V3F velError = velCmd - vel;

  accelCmd.x = kpPosXY * posError.x + kpVelXY * velError.x + accelCmd.x;
  accelCmd.y = kpPosXY * posError.y + kpVelXY * velError.y + accelCmd.y;

  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
  
  return accelCmd;
}

```

#### Function `AltitudeControl()`

```
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  float posZerror = posZCmd - posZ;

  velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);

  float velZerror = velZCmd - velZ;

  float uBar1 = kpPosZ * posZerror + kpVelZ * velZerror + accelZCmd;

  float thrustAcc = (uBar1 - 9.81f) / R(2,2);

  thrust = thrustAcc * mass;
  
  return - thrust;
}
```

#### Tune parameters `Kp_pos_z` and `Kp_vel_z`

#### Tune parameters `Kp_vel_xy` and `Kp_vel_z`

#### Function `YawControl()`

```
float QuadControl::YawControl(float yawCmd, float yaw)
{
  float yawRateCmd=0;

  yawCmd = fmod(yawCmd, (2.0*M_PI));
  float yawError = yawCmd - yaw;

  if (yawError <= -M_PI)
  {
     yawError += (2.0*M_PI);
  }
  else if (yawError > M_PI)
  {
     yawError -= (2.0*M_PI);
  }

  yawRateCmd = kpYaw * yawError;

  return yawRateCmd;
}

```

#### Tune parameters `Kp_yaw` and the 3rd (z) component of `Kp_pqr`



### **2.4:** Non-idealities and robustness

#### Add Integral Control in `AltitudeControl()` 

```
float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  float posZerror = posZCmd - posZ;

  velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);

  float velZerror = velZCmd - velZ;

  integratedAltitudeError += posZerror * dt;

  float maxIntegratedAltitudeError = 0.035f;

  integratedAltitudeError = CONSTRAIN(integratedAltitudeError, -maxIntegratedAltitudeError, maxIntegratedAltitudeError);

  float uBar1 = kpPosZ * posZerror + kpVelZ * velZerror + accelZCmd + KiPosZ * integratedAltitudeError;

  float thrustAcc = (uBar1 - 9.81f) / R(2,2);

  thrust = thrustAcc * mass;
  
  return - thrust;
}
```

### **2.5:** Tracking trajectories

#### Fine Tune


___
## **Task 3:** Extra Challenges

### **Extra Challenge 1:** Improving trajectory generation with target velocities.

```
import math;

def fmt(value):
    return "%.3f" % value

period = [4, 2, 4]
radius = 1.5
timestep = 0.02
maxtime = max(period)*3
timemult = [1, 1, 1]
phase=[0,0,0]
amp = [1,0.4,.5]
center = [0, 0, -2]

with open('FigureEight.txt', 'w') as the_file:
    t=0;
    px = 0;
    py = 0;
    pz = 0;
    while t <= maxtime:

        x = math.sin(t * 2 * math.pi / period[0] + phase[0]) * radius * amp[0] + center[0];
        y = math.sin(t * 2 * math.pi / period[1] + phase[1]) * radius * amp[1] + center[1];
        z = math.sin(t * 2 * math.pi / period[2] + phase[2]) * radius * amp[2] + center[2];
        the_file.write(fmt(t) + "," + fmt(x) + "," + fmt(y) + "," + fmt(z));

        vx = (x-px)/timestep;
        vy = (y-py)/timestep;
        vz = (z-pz)/timestep;
        the_file.write("," + fmt(vx) + "," + fmt(vy) + "," + fmt(vz));
	
        the_file.write("\n");

        px = x;
        py = y;
        pz = z;

        t += timestep;
```

### **Extra Challenge 2:** Improving trajectory generation with minimum snap trajectories.

