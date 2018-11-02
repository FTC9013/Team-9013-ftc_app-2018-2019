package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import java.util.Collections;
import java.util.List;

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
  
  // Robot speed [-1, 1].
  private static double vD;
  // Robot angle while moving [0, 2pi].
  private static double thetaD;
  // Speed for changing direction [-1, 1].
  private static double vTheta;
  
  // comment.....
  
  MecanumDriveChassis(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_F");
    leftRearDrive = hardwareMap.get(DcMotor.class, "left_drive_R");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive_F");
    rightRearDrive = hardwareMap.get(DcMotor.class, "right_drive_R");

    leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
  }


  // These are the methods you need to implement

  // Y=forward, backward movement, X=side to side (strafe), and turn=rotate in place
  void drive(float driveSpeedY, float driveSpeedX, float turn )
  {

    // Math out what to send to the motors.

    // This needs work...

    rightFrontDriveSpeed = Range.clip(-driveSpeedY -driveSpeedX +turn,-1.0,1.0);
    leftFrontDriveSpeed = Range.clip(-driveSpeedY +driveSpeedX -turn,-1.0,1.0);
    rightRearDriveSpeed = Range.clip(-driveSpeedY +driveSpeedX +turn,-1.0,1.0);
    leftRearDriveSpeed = Range.clip(-driveSpeedY -driveSpeedX -turn,-1.0,1.0);



    // send the speeds to the motors
    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);
  }
  
//import java.util.Arrays;


/**
 * Mecanum wheel drive calculations.
 * Input controls:
 *   V_d = desired robot speed.
 *   theta_d = desired robot velocity angle.
 *   V_theta = desired robot rotational speed.
 *
 *  Example:
 *    // Convert joysticks to wheel powers.
 *    Mecanum.Wheels wheels = Mecanum.motionToWheels(
 *        Mecanum.joystickToMotion(
 *            gamepad1.left_stick_x, gamepad1.left_stick_y,
 *            gamepad1.right_stick_x, gamepad1.right_stick_y));
 *    // Set power on the motors.
 *    frontLeftMotor.setPower(wheels.frontLeft);
 */
public class Mecanum {


  /**
   * Gets the motion vector from the joystick values.
   * @param leftStickX The left joystick X.
   * @param leftStickY The left joystick Y.
   * @param rightStickX The right joystick X.
   * @param rightStickY The right joystick Y.
   * @return The Mecanum motion vector.
   */
  public static Motion joystickToMotion(double leftStickX,
                                        double leftStickY,
                                        double rightStickX,
                                        double rightStickY) {
    double vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) +
                                       Math.pow(leftStickY, 2)),
        1);
    double thetaD = Math.atan2(-leftStickX, -leftStickY);
    double vTheta = -rightStickX;
    return new Motion(vD, thetaD, vTheta);
  }

  /**
   * Mecanum wheels, used to get individual motor powers.
   */
  public static class Wheels {
    // The mecanum wheels.
    public final double frontLeft;
    public final double frontRight;
    public final double backLeft;
    public final double backRight;

    /**
     * Sets the wheels to the given values.
     */
    public Wheels(double frontLeft, double frontRight,
                  double backLeft, double backRight) {
      List<Double> powers = Arrays.asList(frontLeft, frontRight,
          backLeft, backRight);
      clampPowers(powers);

      this.frontLeft = powers.get(0);
      this.frontRight = powers.get(1);
      this.backLeft = powers.get(2);
      this.backRight = powers.get(3);
    }

    /**
     * Scales the wheel powers by the given factor.
     * @param scalar The wheel power scaling factor.
     */
    public Wheels scaleWheelPower(double scalar) {
      return new Wheels(frontLeft * scalar, frontRight * scalar,
          backLeft * scalar, backRight * scalar);
    }
  }

  /**
   * Gets the wheel powers corresponding to desired motion.
   * @param motion The Mecanum motion vector.
   * @return The wheels with clamped powers. [-1, 1]
   */
  public static Wheels motionToWheels(Motion motion) {
    double vD = motion.vD;
    double thetaD = motion.thetaD;
    double vTheta = motion.vTheta;

    double frontLeft = vD * Math.sin(-thetaD + Math.PI / 4) - vTheta;
    double frontRight  = vD * Math.cos(-thetaD + Math.PI / 4) + vTheta;
    double backLeft = vD * Math.cos(-thetaD + Math.PI / 4) - vTheta;
    double backRight = vD * Math.sin(-thetaD + Math.PI / 4) + vTheta;
    return new Wheels(frontLeft, frontRight,
        backLeft, backRight);
  }

  /**
   * Clamps the motor powers while maintaining power ratios.
   * @param powers The motor powers to clamp.
   */
  private static void clampPowers(List<Double> powers) {
    double minPower = Collections.min(powers);
    double maxPower = Collections.max(powers);
    double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

    if (maxMag > 1.0) {
      for (int i = 0; i < powers.size(); i++) {
        powers.set(i, powers.get(i) / maxMag);
      }
    }
  }
}




}
