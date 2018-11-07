package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm
{
  private DcMotor armMotor = null;
  
  static final double armRun = 0.70;
  static final double armStop = 0;
  
  //Maximum height of Elevator.
  //static final int top = 72;
  //Minimum height of Elevator.
  //static final int bottom = 0;

  //int curPose = 0;
  //int tarPose = 0;
  
  // comment.....
  
  Arm(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    armMotor = hardwareMap.get(DcMotor.class, "aMotor");
  
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
    armMotor.setDirection(DcMotor.Direction.REVERSE);
  
    armMotor.setPower(armStop);
  
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
    armMotor.setPower(armRun);

    armMotor.setTargetPosition(0);

    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  void lift()
  {
    armMotor.setTargetPosition(armMotor.getCurrentPosition()-4);
    if(armMotor.getTargetPosition() < 0){
      armMotor.setTargetPosition(0);
    }
  }

  void lower()
  {
    armMotor.setTargetPosition(armMotor.getCurrentPosition()+4);
    if(armMotor.getTargetPosition() > 72){
      armMotor.setTargetPosition(72);
    }
  }
}
