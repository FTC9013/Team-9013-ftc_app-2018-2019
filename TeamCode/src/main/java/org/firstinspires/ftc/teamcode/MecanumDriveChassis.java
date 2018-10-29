package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class MecanumDriveChassis
{
  private DcMotor leftDrive = null;
  private DcMotor left2Drive = null;
  private DcMotor rightDrive = null;
  private DcMotor right2Drive = null;

  // comment.....
  
  MecanumDriveChassis(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftDrive  = hardwareMap.get(DcMotor.class, "left_drive_F");
    left2Drive = hardwareMap.get(DcMotor.class,  "left_drive_R" );
    rightDrive = hardwareMap.get(DcMotor.class, "right_drive_F");
    right2Drive = hardwareMap.get(DcMotor.class, "right_drive_R");
  
    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    left2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    right2Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  
    leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    left2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    right2Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
  
    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    leftDrive.setDirection(DcMotor.Direction.REVERSE);
    left2Drive.setDirection(DcMotor.Direction.REVERSE);
    rightDrive.setDirection(DcMotor.Direction.FORWARD);
    right2Drive.setDirection(DcMotor.Direction.FORWARD);
  }
  
  
// These are going away.  Just used as example for testing.
   void setLeftDrive(double leftPower)
   {
     leftDrive.setPower(leftPower);
   }
   void setRightDrive(double rightPower)
   {
     rightDrive.setPower(rightPower);
  
   }
   void setLeft2Drive(double left2Power)
   {
     left2Drive.setPower(left2Power);
   }
   void setRight2Drive(double right2Power)
   {
     right2Drive.setPower( right2Power);
   }

// These are the methods you need to implement
  
  void strafe(double strafeSpeed)
  {
    rightDrive.setPower(Range.clip(strafeSpeed,-1,1));
    leftDrive.setPower(Range.clip(-strafeSpeed,-1,1));
    right2Drive.setPower(Range.clip(-strafeSpeed,-1, 1));
    left2Drive.setPower(Range.clip(strafeSpeed,-1,1));
  }
  
  
  void drive(double driveSpeed)
  {
  
  
  }
  
  
  void drive(double driveSpeed, int distance)
  {
  
  
  }
  
  
  void drive(double driveSpeed, Boolean absRel, int angle)
  {
  
  
  }
  
  
  void drive(double driveSpeed, Boolean absRel, int angle, int distance)
  {
  
  
  }
  
  
  
}
