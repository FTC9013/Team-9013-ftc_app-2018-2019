package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Collections;
import java.util.List;
import java.util.Arrays;

import static android.os.SystemClock.sleep;


public class MecanumDriveChassisAutonomousIMU
{
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;
  private BNO055IMU imu = null;


  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;

  // Robot speed [-1, 1].
  private static double vD;
  // Robot angle while moving [0, 2pi].
  private static double thetaD;
  // Speed for changing direction [-1, 1].
  private static double vTheta;

  // Robot speed scaling factor
  private final float speedScale = 2;

  MecanumDriveChassisAutonomousIMU(HardwareMap hardwareMap)
  {
    // get amd initialize the DC Motors.
    leftFrontDrive = hardwareMap.get(DcMotor.class, "lFront"); //hub 3 port 0
    leftRearDrive = hardwareMap.get(DcMotor.class, "lRear"); //hub 3 port 2
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rFront"); //hub 3 port 1
    rightRearDrive = hardwareMap.get(DcMotor.class, "rRear"); //hub 3 port 3

    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

    // set motion parameters.
    vD = 0;
    thetaD = 0;
    vTheta = 0;

    // Set all the motor speeds to zero.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;

    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);


    // Get and initialize the IMU. (we will use the imu on hub id = 3)
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;

    imu.initialize(parameters);


    // make sure the imu gyro is calibrated before continuing.

    // THIS WILL have to be sorted out...

/*    while (!isStopRequested() && !imu.isGyroCalibrated())
    {
      sleep(50);
      idle();
    }*/
  }


  // X=side to side (strafe), Y=forward, backward movement, and turn=rotate in place
  void drive(float driveSpeedX, float driveSpeedY, float turn )
  {
    // calculate the vectors
    joystickToMotion( driveSpeedX/speedScale, driveSpeedY/speedScale,
        turn/speedScale  );

    // Math out what to send to the motors and send it.
    PowerToWheels();
  }

  /**
   * Process the joystick values into motion vector.
   *  V_d = desired robot speed.
   *  theta_d = desired robot velocity angle.
   *  V_theta = desired robot rotational speed.
   */
  private void joystickToMotion( double leftStickX,
                                 double leftStickY,
                                 double rightStickX ) {
    vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2)), 1);
    thetaD = Math.atan2(-leftStickX, -leftStickY);
    vTheta = -rightStickX;
  }

  /**
  * Calculate the power settings and send to the wheels
  */
  private void PowerToWheels() {

    rightFrontDriveSpeed = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
    leftFrontDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
    rightRearDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;
    leftRearDriveSpeed   = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;

    // scales the motor powers while maintaining power ratios.
    List<Double> speeds = Arrays.asList(rightFrontDriveSpeed,
        leftFrontDriveSpeed, rightRearDriveSpeed, leftRearDriveSpeed  );

    double minPower = Collections.min(speeds);
    double maxPower = Collections.max(speeds);
    double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

    if (maxMag > 1.0)
    {
      for (int i = 0; i < speeds.size(); i++)
      {
        speeds.set(i, speeds.get(i) / maxMag);
      }
    }
    // must be same order as placed in the list
    rightFrontDriveSpeed = speeds.get(0);
    leftFrontDriveSpeed  = speeds.get(1);
    rightRearDriveSpeed  = speeds.get(2);
    leftRearDriveSpeed   = speeds.get(3);

    // send the speeds to the motors
    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);
  }

  /**
  *
  */
  void drive(float driveSpeedY, int distance)
  {


  }

  /**
  *
  */
  void drive(float driveSpeedY, Boolean absRel, int angle)
  {


  }

  /**
  *
  */
  void drive(float driveSpeedY, Boolean absRel, int angle, int distance)
  {


  }
}

//
//  // Use gyro to drive in a straight line.
//  correction = checkDirection();
//
//            telemetry.addData("1 imu heading", lastAngles.firstAngle);
//            telemetry.addData("2 global heading", globalAngle);
//            telemetry.addData("3 correction", correction);
//            telemetry.update();
//
//            leftMotor.setPower(-power + correction);
//            rightMotor.setPower(-power);
//
//  // We record the sensor values because we will test them in more than
//  // one place with time passing between those places. See the lesson on
//  // Timing Considerations to know why.
//
//  aButton = gamepad1.a;
//  bButton = gamepad1.b;
//  touched = touch.getState();
//
//            if (!touched || aButton || bButton)
//  {
//    // backup.
//    leftMotor.setPower(power);
//    rightMotor.setPower(power);
//
//    sleep(500);
//
//    // stop.
//    leftMotor.setPower(0);
//    rightMotor.setPower(0);
//
//    // turn 90 degrees right.
//    if (!touched || aButton) rotate(-90, power);
//
//    // turn 90 degrees left.
//    if (bButton) rotate(90, power);
//  }
//}
//
//// turn the motors off.
//        rightMotor.setPower(0);
//            leftMotor.setPower(0);
//            }
//
///**
// * Resets the cumulative angle tracking to zero.
// */
//private void resetAngle()
//            {
//            lastAngles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
//
//            globalAngle=0;
//            }
//
///**
// * Get current cumulative angle rotation from last reset.
// *
// * @return Angle in degrees. + = left, - = right.
// */
//private double getAngle()
//            {
//            // We experimentally determined the Z axis is the axis we want to use for heading angle.
//            // We have to process the angle because the imu works in euler angles so the Z axis is
//            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//            Orientation angles=imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);
//
//            double deltaAngle=angles.firstAngle-lastAngles.firstAngle;
//
//            if(deltaAngle< -180)
//                               deltaAngle+=360;
//                               else if(deltaAngle>180)
//                               deltaAngle-=360;
//
//                               globalAngle+=deltaAngle;
//
//                               lastAngles=angles;
//
//                               return globalAngle;
//                               }
//
///**
// * See if we are moving in a straight line and if not return a power correction value.
// * @return Power adjustment, + is adjust left - is adjust right.
// */
//private double checkDirection()
//            {
//            // The gain value determines how sensitive the correction is to direction changes.
//            // You will have to experiment with your robot to get small smooth direction changes
//            // to stay on a straight line.
//            double correction,angle,gain=.10;
//
//            angle=getAngle();
//
//            if(angle==0)
//            correction=0;             // no adjustment.
//            else
//            correction=-angle;        // reverse sign of angle for correction.
//
//            correction=correction*gain;
//
//            return correction;
//            }
//
///**
// * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
// * @param degrees Degrees to turn, + is left - is right
// */
//private void rotate(int degrees,double power)
//            {
//            double leftPower,rightPower;
//
//            // restart imu movement tracking.
//            resetAngle();
//
//            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//            // clockwise (right).
//
//            if(degrees< 0)
//   {   // turn right.
//   leftPower=-power;
//   rightPower=power;
//   }
//   else if(degrees>0)
//   {   // turn left.
//   leftPower=power;
//   rightPower=-power;
//   }
//   else return;
//
//   // set power to rotate.
//   leftMotor.setPower(leftPower);
//   rightMotor.setPower(rightPower);
//
//   // rotate until turn is completed.
//   if(degrees< 0)
//    {
//    // On right turn we have to get off zero first.
//    while(opModeIsActive()&&getAngle()==0){}
//
//    while(opModeIsActive()&&getAngle()>degrees){}
//    }
//    else    // left turn.
//    while(opModeIsActive()&&getAngle()<degrees){}
//
//               // turn the motors off.
//               rightMotor.setPower(0);
//               leftMotor.setPower(0);
//
//               // wait for rotation to stop.
//               sleep(1000);
//
//               // reset angle tracking on new heading.
//               resetAngle();
//               }
//

