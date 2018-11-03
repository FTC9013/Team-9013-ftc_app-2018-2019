package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm
{
  private DcMotor armMotor = null;
  
  static final double armRun = 0.25;
  static final double armStop = 0;
  
  //Maximum height of Elevator.
  static final int top = 72;
  //Minimum height of Elevator.
  static final int bottom = 0;
  
  
  // comment.....
  
  Arm(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    armMotor = hardwareMap.get(DcMotor.class, "ElevatorMotor");
  
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
    armMotor.setDirection(DcMotor.Direction.FORWARD);
  
    armMotor.setPower(armStop);
  
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
    armMotor.setPower(armRun);
  
    armMotor.setTargetPosition(bottom);
  
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  void lift()
  {
    armMotor.setTargetPosition(top);
  }
  
  void lower()
  {
    armMotor.setTargetPosition(bottom);
  }
}
