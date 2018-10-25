package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Collector
{
  private DcMotor leftDrive = null;


  // comment.....
  
  Collector(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftDrive  = hardwareMap.get(DcMotor.class, "left_drive_F");

  
    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

  
    leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

  
  
    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    leftDrive.setDirection(DcMotor.Direction.REVERSE);

  }
  
  
  
   void setLeftDrive(double leftPower)
   {
     leftDrive.setPower(leftPower);
   }
   
   
  
}
