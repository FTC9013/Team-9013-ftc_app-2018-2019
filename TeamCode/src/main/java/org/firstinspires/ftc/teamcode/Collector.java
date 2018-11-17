package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collector
{
  private Servo collectorServo = null;
  // comment.....
  
  Collector(HardwareMap hardwareMap)
  {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    collectorServo = hardwareMap.servo.get("cServo");
    collectorServo.setDirection(Servo.Direction.FORWARD);
  }
  
  
  void collect()
  {
    collectorServo.setPosition(0);
  }
  
  void drop()
  {
    collectorServo.setPosition(1);
  }
  
  void cancel() {
    collectorServo.setPosition(0.5);
  }
  
}