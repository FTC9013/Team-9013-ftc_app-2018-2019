package org.firstinspires.ftc.teamcode;

// mode:     {FORWARD, BACKWARDS, LEFT, RIGHT, TURN_DRIVE}
// speed:    the drive speed from 0-100%
// angle:    the desired angle of travel relative to the current bot position and orientation.
//           in DEGREES
// distance: the distance to travel in inches.

public class Leg
{
  public enum Mode {FORWARD, BACKWARDS, LEFT, RIGHT, TURN_DRIVE}
  public Mode mode;
  public double speed;
  public double angle;
  public double distance;
  
  
  Leg( Mode mode, double speed, double angle, double distance){
    this.mode = mode;
    this.speed = speed;
    this.angle = angle;
    this.distance = distance;
  }
}
