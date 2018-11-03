package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
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

  /**
   * Process the joystick values into motion vector.
   *  V_d = desired robot speed.
   *  theta_d = desired robot velocity angle.
   *  V_theta = desired robot rotational speed.
   */
  void joystickToMotion( double leftStickX,
                         double leftStickY,
                         double rightStickX ) {
    vD = Math.min(Math.sqrt(Math.pow(leftStickX, 2) + Math.pow(leftStickY, 2)), 1);
    thetaD = Math.atan2(-leftStickX, -leftStickY);
    vTheta = -rightStickX;
  }

  /**
     * Scales the wheel powers by the given factor.
     * @param scalar The wheel power scaling factor.
  */
  void scalePower(double scalar) {
    // Scale all the motor speeds.
    rightFrontDriveSpeed = rightFrontDriveSpeed * scalar;
    leftFrontDriveSpeed  = leftFrontDriveSpeed  * scalar;
    rightRearDriveSpeed  = rightRearDriveSpeed  * scalar;
    leftRearDriveSpeed   = leftRearDriveSpeed   * scalar;
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
