package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm
{
  private DcMotor armMotor = null;
  
  static final double armRun = 1.0;
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
    armMotor = hardwareMap.get(DcMotor.class, "aMotor");  //hub 2 port 1
    
    armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
    armMotor.setDirection(DcMotor.Direction.REVERSE);
  
    armMotor.setPower(armStop);
  
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
    armMotor.setPower(armRun);

    armMotor.setTargetPosition(0);

    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  ElapsedTime armTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private final double armHoldOffTime = 0.5;
  
  public boolean armMoveAllowed(){
    return armTimer.time() > armHoldOffTime;
  }
  void lift()
  {
    if(armMoveAllowed()){
      armMotor.setTargetPosition(armMotor.getCurrentPosition()-12);
      if(armMotor.getTargetPosition() < 0){
        armMotor.setTargetPosition(0);
      }
    }
  }

  void lower()
  {
    if(armMoveAllowed()){
      armMotor.setTargetPosition(armMotor.getCurrentPosition()+12);
      if(armMotor.getTargetPosition() > 72){
        armMotor.setTargetPosition(72);
      }
    }
  }
}
