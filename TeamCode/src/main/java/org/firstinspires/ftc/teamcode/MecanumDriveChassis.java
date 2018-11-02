package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDriveChassis
{
  private DcMotor leftFrontDrive = null;
  private DcMotor leftRearDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor rightRearDrive = null;


  private double leftFrontDriveSpeed;
  private double leftRearDriveSpeed;
  private double rightFrontDriveSpeed;
  private double rightRearDriveSpeed;

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
