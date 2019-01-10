package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static java.lang.Math.abs;

public class Elevator
{
  private DcMotor elevatorMotor = null;

  static final double elevatorRun = 1.0;
  static final double elevatorStop = 0;

  //Maximum height of Elevator.
  static final int topFloor = 1175;
  //Minimum height of Elevator.
  static final int lobbyFloor = 0;

  // +/- counts to say the motor is stopped a the target position.
  static final int closeEnough = 10;
  
  //Reset lowering of Elevator.
  static final int resetDown = -1150;
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
  
    void up()
    {
      // send the elevator up (land the bot)
      elevatorMotor.setTargetPosition(topFloor);
      // wait for the elevator to get to the target
      while( isMoving());
    }

    void down()
    {
      // send the elevator down (raise the bot)
      elevatorMotor.setTargetPosition(lobbyFloor);
      // wait for the elevator to get to the target
      while( isMoving());
    }

    boolean isMoving()   {
      // is the current position is within 'closeEnough' counts of the set point position
      // could use the motor method isBusy but not sure how it works... may be too sensitive.
      return abs( elevatorMotor.getCurrentPosition()
                 - elevatorMotor.getTargetPosition()) > closeEnough;
    }

    // used by the LowerElevator Opmode to move the elevator down if
    // it is left up for some reason
    void resetDown()   {
      elevatorMotor.setTargetPosition(resetDown);
    }
}
