package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class Arm
{
  private DcMotorEx  armMotor = null;
  
  static final double armRun = 1.0;
  static final double armStop = 0;
  
  static final double newP = 20;
  static final double newI = 1;
  static final double newD = 3;
  static final double newF = 0;
  
  // Golden PIDF: P = 20, I = 1, D = 3, F = 0
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
    armMotor = (DcMotorEx)hardwareMap.get(DcMotor.class, "aMotor");  //hub 2 port 1
    
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
    armMotor.setDirection(DcMotor.Direction.FORWARD);
  
    armMotor.setPower(armStop);
  
    // get the PID coefficients for the RUN_USING_ENCODER  modes.
    PIDFCoefficients pidOrig = armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
  
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
    armMotor.setPower(armRun);

    armMotor.setTargetPosition(0);
  
    // change coefficients using methods included with DcMotorEx class.
    PIDFCoefficients pidNew = new PIDFCoefficients( newP, newI, newD, newF );
    armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
  
    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
  }
  
  void lift()
  {
    armMotor.setTargetPosition(230);
  }

  void lower()
  {
    armMotor.setTargetPosition(50);
  }
  
  void drop()
  {
    armMotor.setTargetPosition(20);
  }
  
 // void getArmPosition(){
  //  return armMotor.getcurre();
 // }
  
  
  
  int getPosition()
  {
    return armMotor.getCurrentPosition();
  }
  
  PIDFCoefficients getPIDFcoefficients()
  {
    return armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
}

