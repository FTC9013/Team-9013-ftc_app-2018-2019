package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumDriveChassisWithIMU {
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;


  private double leftFrontDriveSpeed;
  private double leftRearDriveSpeed;
  private double rightFrontDriveSpeed;
  private double rightRearDriveSpeed;

  private BNO055IMU imu;
  private Orientation lastAngles = new Orientation();

  // comment.....

  MecanumDriveChassisWithIMU(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_F");
    leftRearDrive = hardwareMap.get(DcMotor.class, "left_drive_R");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive_F");
    rightRearDrive = hardwareMap.get(DcMotor.class, "right_drive_R");

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

    // Set all the motor speeds to zero.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;

    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    parameters.mode = BNO055IMU.SensorMode.IMU;
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = false;

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    imu.initialize(parameters);


    // make sure the imu gyro is calibrated before continuing.

    // THIS WILL have to be sorted out...

//    while (!isStopRequested() && !imu.isGyroCalibrated())
//    {
//      sleep(50);
//      idle();
//    }
  }


  // These are the methods you need to implement

  // Y=forward, backward movement, X=side to side (strafe), and turn=rotate in place
  void drive(float driveSpeedY, float driveSpeedX, float turn)
  {

    // Math out what to send to the motors.

    // This needs work...

    rightFrontDriveSpeed = Range.clip(-driveSpeedY - driveSpeedX + turn, -1.0, 1.0);
    leftFrontDriveSpeed = Range.clip(-driveSpeedY + driveSpeedX - turn, -1.0, 1.0);
    rightRearDriveSpeed = Range.clip(-driveSpeedY + driveSpeedX + turn, -1.0, 1.0);
    leftRearDriveSpeed = Range.clip(-driveSpeedY - driveSpeedX - turn, -1.0, 1.0);


    // send the speeds to the motors
    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);
  }


  void drive(float driveSpeedY, int distance)
  {


  }


  void drive(float driveSpeedY, Boolean absRel, int angle)
  {


  }


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
//import java.util.Arrays;
//    import java.util.Collections;
//    import java.util.List;
//
///**
// * Mecanum wheel drive calculations.
// * Input controls:
// *   V_d = desired robot speed.
// *   theta_d = desired robot velocity angle.
// *   V_theta = desired robot rotational speed.
// *
// *  Example:
// *    // Convert joysticks to wheel powers.
// *    Mecanum.Wheels wheels = Mecanum.motionToWheels(
// *        Mecanum.joystickToMotion(
// *            gamepad1.left_stick_x, gamepad1.left_stick_y,
// *            gamepad1.right_stick_x, gamepad1.right_stick_y));
// *    // Set power on the motors.
// *    frontLeftMotor.setPower(wheels.frontLeft);
// */
//public class Mecanum {
//  /**
//   * Mecanum motion vector.
//   */
//  public static class Motion {
//    // Robot speed [-1, 1].
//    public final double vD;
//    // Robot angle while moving [0, 2pi].
//    public final double thetaD;
//    // Speed for changing direction [-1, 1].
//    public final double vTheta;
//
//    /**
//     * Sets the motion to the given values.
//     */
//    public Motion(double vD, double thetaD, double vTheta) {
//      this.vD = vD;
//      this.thetaD = thetaD;
//      this.vTheta = vTheta;
//    }
//  }
//
//  /**
//   * Gets the motion vector from the joystick values.
//   * @param leftStickX The left joystick X.
//   * @param leftStickY The left joystick Y.
//   * @param rightStickX The right joystick X.
//   * @param rightStickY The right joystick Y.
//   * @return The Mecanum motion vector.
//   */
//  public static Motion joystickToMotion(double leftStickX,
//                                        double leftStickY,
//                                        double rightStickX,
//                                        double rightStickY) {
//    double vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) +
//                                       Math.pow(leftStickY, 2)),
//        1);
//    double thetaD = Math.atan2(-leftStickX, -leftStickY);
//    double vTheta = -rightStickX;
//    return new Motion(vD, thetaD, vTheta);
//  }
//
//  /**
//   * Mecanum wheels, used to get individual motor powers.
//   */
//  public static class Wheels {
//    // The mecanum wheels.
//    public final double frontLeft;
//    public final double frontRight;
//    public final double backLeft;
//    public final double backRight;
//
//    /**
//     * Sets the wheels to the given values.
//     */
//    public Wheels(double frontLeft, double frontRight,
//                  double backLeft, double backRight) {
//      List<Double> powers = Arrays.asList(frontLeft, frontRight,
//          backLeft, backRight);
//      clampPowers(powers);
//
//      this.frontLeft = powers.get(0);
//      this.frontRight = powers.get(1);
//      this.backLeft = powers.get(2);
//      this.backRight = powers.get(3);
//    }
//
//    /**
//     * Scales the wheel powers by the given factor.
//     * @param scalar The wheel power scaling factor.
//     */
//    public Wheels scaleWheelPower(double scalar) {
//      return new Wheels(frontLeft * scalar, frontRight * scalar,
//          backLeft * scalar, backRight * scalar);
//    }
//  }
//
//  /**
//   * Gets the wheel powers corresponding to desired motion.
//   * @param motion The Mecanum motion vector.
//   * @return The wheels with clamped powers. [-1, 1]
//   */
//  public static Wheels motionToWheels(Motion motion) {
//    double vD = motion.vD;
//    double thetaD = motion.thetaD;
//    double vTheta = motion.vTheta;
//
//    double frontLeft = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
//    double frontRight  = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
//    double backLeft = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;
//    double backRight = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;
//    return new Wheels(frontLeft, frontRight,
//        backLeft, backRight);
//  }
//
//  /**
//   * Clamps the motor powers while maintaining power ratios.
//   * @param powers The motor powers to clamp.
//   */
//  private static void clampPowers(List<Double> powers) {
//    double minPower = Collections.min(powers);
//    double maxPower = Collections.max(powers);
//    double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));
//
//    if (maxMag > 1.0) {
//      for (int i = 0; i < powers.size(); i++) {
//        powers.set(i, powers.get(i) / maxMag);
//      }
//    }
//  }
//}
