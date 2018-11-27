/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
Motor Names for Configuration

Hub 2:
0 = eMotor (rev robotics 40:1 HD Hex)
1 = aMotor (rev robotics 40:1 HD Hex)
0 = cServo

Hub 3:

0 = lFront (NeveRest 40 Gearmotor)
1 = rFront (NeveRest 40 Gearmotor)
2 = lRear (NeveRest 40 Gearmotor)
3 = rRear (NeveRest 40 Gearmotor)
*/

@TeleOp(name="Primary Tele-Op", group="Linear Opmode")
//@Disabled
public class TeleOpPrimary extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassis driveChassis;
  private Elevator landingElevator;
  private Arm collectorArm;
  private Collector collector;
  
  private ElapsedTime runtime = new ElapsedTime();
  // a timer for debouncing all the buttons on the game pad that need debounce.
  ElapsedTime bounceTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  // the time to hold before allowing state change on button.
  private final double buttonDebounceTime = 0.10;
  // debounce lockout variables.
  private double gamepad1XDebounceLockTime = 0;


  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    driveChassis = new MecanumDriveChassis(hardwareMap);
    landingElevator = new Elevator(hardwareMap);
    collectorArm = new Arm(hardwareMap);
    collector = new Collector(hardwareMap);

    boolean gamepad1XToggleFlag = false;
    boolean gamepad1XToggleLock = false;
    boolean elevatorUp = false;

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // send joystick inputs to the drive chassis
      driveChassis.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
      
      if(gamepad1XToggleFlag && !elevatorUp) {
        landingElevator.up();
        elevatorUp = true;
      }
      else if(!gamepad1XToggleFlag && elevatorUp){
        landingElevator.down();
        elevatorUp = false;
      }

      if(gamepad2.dpad_up) {
        collectorArm.lift();
      }
      if(gamepad2.dpad_down) {
        collectorArm.lower();
      }

      if(gamepad2.x && !gamepad1XToggleFlag && !gamepad1XToggleLock
         && bounceTimeCheck(gamepad1XDebounceLockTime)){
        gamepad1XToggleFlag = true;
        gamepad1XToggleLock = true;
        gamepad1XDebounceLockTime = bounceTimer.time();
      }
      else if(gamepad2.x && gamepad1XToggleFlag && !gamepad1XToggleLock
              && bounceTimeCheck(gamepad1XDebounceLockTime)){
        gamepad1XToggleFlag = false;
        gamepad1XToggleLock = true;
        gamepad1XDebounceLockTime = bounceTimer.time();
      }
      if(!gamepad2.x && bounceTimeCheck(gamepad1XDebounceLockTime)) {
        gamepad1XToggleLock = false;
        gamepad1XDebounceLockTime = bounceTimer.time();
      }
      
      if(gamepad1.right_bumper){
        collector.collect();
      }
      else if(gamepad1.left_bumper){
        collector.drop();
      }
      else{
        collector.cancel();
      }
      
      
      // Show the elapsed game time and wheel power.
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Stick", "Y_left (%.2f), X_left (%.2f)",
                        gamepad1.left_stick_y, gamepad1.left_stick_x);
      telemetry.update();
    }
  }

  // test the debounce lockout against the timer
  private boolean bounceTimeCheck ( double lockoutVariable){
    return bounceTimer.time() > (lockoutVariable + buttonDebounceTime);
  }
}
