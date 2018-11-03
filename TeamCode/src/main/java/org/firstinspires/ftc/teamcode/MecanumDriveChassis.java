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
  
  // Robot speed [-1, 1].
  private static double vD;
  // Robot angle while moving [0, 2pi].
  private static double thetaD;
  // Speed for changing direction [-1, 1].
  private static double vTheta;
  
  // Robot speed scaling factor
  private final float speedScale = 2;
  
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
}
