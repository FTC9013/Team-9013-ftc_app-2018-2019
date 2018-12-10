package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
public class Elevator
{
  private DcMotor elevatorMotor = null;

  static final double elevatorRun = 1.0;
  static final double elevatorStop = 0;

  //Maximum height of Elevator.
  static final int topFloor = 1100;
  //Minimum height of Elevator.
  static final int lobbyFloor = 0;


  // comment.....
  
  Elevator(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    elevatorMotor = hardwareMap.get(DcMotor.class, "eMotor");  //hub 2 port 0

    elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

    elevatorMotor.setPower(elevatorStop);

    elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    elevatorMotor.setPower(elevatorRun);

    elevatorMotor.setTargetPosition(lobbyFloor);

    elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
   void up()   {
       
         elevatorMotor.setTargetPosition(topFloor);
       
    
   }

   void down()   {
    
       elevatorMotor.setTargetPosition(lobbyFloor);
     
     
   }
}
