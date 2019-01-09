package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Collections;
import java.util.List;
import java.util.Arrays;
import java.util.Locale;

public class MecanumDriveChassisAutonomousIMU
{
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;
  private BNO055IMU imu = null;

  // State used for updating telemetry
  Orientation angles;
  Acceleration gravity;

  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;
  
  private static double translateAngle;
  private static double translateDistance;
  private static double driveDistance;
  private static double rotateToAngle;
  private static float startAngle;
  private static boolean driveMode;  // leg drive mode
  private static boolean moving;
  
  public static IMUTelemetry IMUTel;
  
  
  // Robot speed [-1, 1].  (speed in any direction that is not rotational)
  // does not have any angular component, just scaler velocity.
  // combined with the angular component for motion.  Even if angle is 0 (forward).
  private static double vD;

  // Robot angle while moving [0, 2PI] or [0, +/-PI]. (angle to displace the center of the bot,
  // ASDF)
  // relative to the direction the bot is facing.
  private static double thetaD;

  // Speed for changing direction [-1, 1] (tank drive style rotate, mouse)
  private static double vTheta;

  // Robot speed scaling factor (% of joystick input to use)
  // applied uniformly across all joystick inputs to the JoystickTokMotion() method.
  private final double speedScale = 0.8;
  
  // speed for a turn
  private final double turnSpeed = 0.3;

  MecanumDriveChassisAutonomousIMU(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
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
    // A positive power number should drive the robot forward regardless of the motor's
    // position on the robot.
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

    // set motion parameters.
    vD = 0;
    thetaD = 0;
    vTheta = 0;

    // Set all the motor speeds.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;

    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);

    
    // start of temporary kludge ******************
    // allows quick shuttle by encoder after landing the bot.
  
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
    leftFrontDrive.setTargetPosition(0);
    leftRearDrive.setTargetPosition(0);
    rightFrontDrive.setTargetPosition(0);
    rightRearDrive.setTargetPosition(0);
    
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // end of temporary kludge ******************
    
    
    // Get and initialize the IMU. (we will use the imu on hub id = 3)
    imu = hardwareMap.get(BNO055IMU.class, "imu1");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = false;
//    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu.initialize(parameters);
  
    IMUTel = new IMUTelemetry();
  }


  boolean IMU_IsCalibrated () {
    return imu.isGyroCalibrated();
  }
  
  /**
   * Calculate the power settings and send to the wheels.  This also translates the force
   * for the Mecanum wheels to the rotated axis based on the degree of the wheel offset.
   * In our case 45 degrees or PI/4
   *
   * Assumes X is forward and Z is up then rotate XY PI/4 to align with wheel axises.
   * placing the positive X axis on the left front wheel and the positive Y axis on the
   * left rear wheel.
   *
   * Rotation is about a positive Z axis pointing UP.
   * Positive Y is to the left.
   *
   * Translation angle is in radians + is CCW - is CW with ZERO to the forward of the bot.
   * I.e. standard rotation about a positive Z axis pointing UP.
   * E.g:
   *    0     = forward
   *    PI/4  = 45 deg. forward and to the left
   *    PI/2  = to the left
   *    3PI/4 = 135 deg. backward and to the left
   *    PI    = backwards
   *
   *    -PI/4  (or) 7PI/4 = 45 deg. forward and to the right
   *    -PI/2  (or) 6PI/4 = to the right
   *    -3PI/4 (or) 5PI/4 = -135 deg. backward and to the left
   *    -PI    (or) PI    = backwards
   *
   * vTheta rotation is also standard rotation about a positive Z axis pointing UP.
   * thus a positive vTheta will turn the bot CCW about its Z axis.
   *
   **/
  private void PowerToWheels() {

    // Motors power = Y component of directional vector
    leftFrontDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
    rightRearDriveSpeed  = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;

    // Motors power = X component of directional vector
    rightFrontDriveSpeed = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
    leftRearDriveSpeed   = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;

    // place all the power numbers in a list for collection manipulations
    // (easier to find min / max etc when in a list)
    List<Double> speeds = Arrays.asList(rightFrontDriveSpeed,
        leftFrontDriveSpeed, rightRearDriveSpeed, leftRearDriveSpeed  );

    // scales the motor powers while maintaining power ratios.
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
    // send the speeds to the motors
    rightFrontDrive.setPower(speeds.get(0));
    leftFrontDrive.setPower(speeds.get(1));
    rightRearDrive.setPower(speeds.get(2));
    leftRearDrive.setPower(speeds.get(3));
  }

  /**
   * this is called every pass of the opmode while() loop to update power to the wheels
   * and return the current telemetry.
   * @return
   */
  IMUTelemetry drive () {
  
  angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
  
  if(driveMode) { // turn and drive

    
  
  
  }
  else {  // translating
  
  }
    
    // Magic Goes Here!
  
    
    // Math out what to send to the motors and send it.
    //PowerToWheels();
    composeTelemetry();
    return IMUTel;
  }
  
  /**
   * returns ture if the bot is still moving
   */
  boolean isMoving() {
//    return moving;
  
    return rightFrontDrive.isBusy();
  }
  
  
  /**
   * loads a movement leg into the target variables.
   */
  void move( Leg leg ) {


    // start of temporary kludge ******************
    // allows quick shuttle by encoder after landing the bot.
  
  
    rightFrontDrive.setPower(turnSpeed);
    leftFrontDrive.setPower(turnSpeed);
    rightRearDrive.setPower(turnSpeed);
    leftRearDrive.setPower(turnSpeed);
    
    leftFrontDrive.setTargetPosition(-500);
    leftRearDrive.setTargetPosition(500);
    rightFrontDrive.setTargetPosition(500);
    rightRearDrive.setTargetPosition(-500);
  
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    // end of temporary kludge ******************

  
  
  
  
  
  
  
  
    // reset start angle from gyro
    // startAngle = angles.firstAngle;  // zero reference for the leg
    // should be piecewise...
    
    if(driveMode) {    // is it turn and drive (true)?
      rotateToAngle = leg.angle;
      driveDistance = leg.distance;
    }
    else {    // is translate
      translateAngle = leg.angle;
      translateDistance = leg.distance;
    }
  }

  //----------------------------------------------------------------------------------------------
  // Telemetry Configuration
  //----------------------------------------------------------------------------------------------

  void composeTelemetry() {
 
    IMUTel.imuStatus = imu.getSystemStatus().toShortString();
    IMUTel.calStatus = imu.getCalibrationStatus().toString();
    IMUTel.zTheta = String.format(Locale.getDefault(), "%.2f", angles.firstAngle);
    IMUTel.yTheta = "!";
    IMUTel.xTheta = "!";
//    IMUTel.zTheta = formatAngle(angles.angleUnit, angles.firstAngle);
//    IMUTel.yTheta = formatAngle(angles.angleUnit, angles.secondAngle);
//    IMUTel.xTheta = formatAngle(angles.angleUnit, angles.thirdAngle);
  
  }
}

