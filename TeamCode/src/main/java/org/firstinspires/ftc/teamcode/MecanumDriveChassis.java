package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Collections;
import java.util.List;
import java.util.Arrays;


public class MecanumDriveChassis
{
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;


  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;
  
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
  // applied uniformly across all joystick inputs to the JoystickToMotion() method.
  private final double speedScale = 0.8;
  
  MecanumDriveChassis(HardwareMap hardwareMap)
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
  }

  // Left  Y = forward, backward movement
  // Left  X = side to side (strafe)
  // Right X = rotate in place
  void drive(float driveLeftY, float driveLeftX, float driveRightX )
  {
    // calculate the vectors multiply input values by scaling factor for max speed.
    joystickToMotion( driveLeftY * speedScale, driveLeftX * speedScale,
        driveRightX * speedScale  );
    
    // Math out what to send to the motors and send it.
    PowerToWheels();
  }

  /**
   * Process the joystick values into motion vector.
   *  Converts the left stick X, Y input into the translation angle and speed
   *  Converts the right stick X axis into rotation speed.
   *
   *  Overall this makes the joysticks like mouse & keyboard game controls with
   *  the left stick acting as the WASD keys and the right stick as the mouse.
   *
   *  vD = desired robot translation speed.
   *  thetaD = desired robot translation angle.
   *  vTheta = desired robot rotational speed.
   */
  private void joystickToMotion( double leftStickY, double leftStickX, double rightStickX ) {
    // determines the translation speed by taking the hypotenuse of the vector created by
    // the X & Y components.
    vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(-leftStickY, 2)), 1);

    // Converts the joystick inputs from cartesian to polar from 0 to +/- PI oriented
    // with 0 to the right of the robot. (standard polar plot)
    thetaD = Math.atan2(-leftStickY, leftStickX);
    // orient to the robot by rotating PI/2 to make the joystick zero at the forward of bot.
    // instead of the right side.
    thetaD = thetaD - Math.PI / 2;
    // simply takes the right stick X value and invert to use as a rotational speed.
    // inverted since we want CW rotation on a positive value.
    // which is opposite of what PowerToWheels() wants in polar positive rotation (CCW).
    vTheta = -rightStickX;
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
}
