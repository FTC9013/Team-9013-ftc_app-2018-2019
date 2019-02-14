package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Arm
{
  private DcMotorEx  armMotor = null;
//  private ModernRoboticsTouchSensor armSensor = null;
  
  static final double armRun = 1.0;
  static final double armStop = 0;
  
  static final double newP = 5;
  static final double newI = 0;
  static final double newD = 0;
  static final double newF = 0;
  
  //Maximum height of arm.
  static final int up = 1000;
  //Minimum height of arm
  static final int down = 10;
  //Height arm will go up in auto
  static final int autoHight = 450;

  
  Arm(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
//    armSensor = (ModernRoboticsTouchSensor)hardwareMap.get(TouchSensor.class, "armSensor"); // Hub 2 MR Touch Sensor port 0
    
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
    armMotor.setTargetPosition(up);
  }

  void lower()
  {
    armMotor.setTargetPosition(down);
  }
  
  void crater() {armMotor.setTargetPosition(autoHight);}
  
  int getPosition()
  {
    return armMotor.getCurrentPosition();
  }
  
  PIDFCoefficients getPIDFcoefficients()
  {
    return armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
}

