package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


class IMU_TEST
{
  private BNO055IMU imu;
  private static IMUTelemetry IMUTelemetry;

  private Orientation angles; // stores the current orientation of the bot from the IMU

  IMU_TEST(HardwareMap hardwareMap)
  {
    // Get and initialize the IMU. (we will use the imu on hub id = 3)
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    IMUTelemetry = new IMUTelemetry();

    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled      = false;

    imu.initialize(parameters);
  
  }

  boolean IMU_IsCalibrated () {
    return imu.isGyroCalibrated();
  }


  IMUTelemetry testAngle(double desiredAngle)
  {
    // desired angle in degrees +/- 0 to 180 where CCW is + and CW is -
    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    IMUTelemetry.headding = angles.firstAngle;
    IMUTelemetry.error = desiredAngle - angles.firstAngle;
    if(IMUTelemetry.error > 180 ) {IMUTelemetry.error -= 360;}
    if(IMUTelemetry.error < -180 ) {IMUTelemetry.error += 360;}

    return IMUTelemetry;

  }
}
