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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaRoverRuckus;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.util.LinkedList;
import java.util.Queue;

@Autonomous(name = "AutonomousPrimary", group = "Linear Opmode")

//@Disabled
public class AutonomousPrimary extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassisAutonomousIMU driveChassis;
  private IMUTelemetry IMUTel;
  private Elevator landingElevator;
  private ElapsedTime runtime = new ElapsedTime();

  VuforiaLocalizer vuforia;
  
  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomousIMU(hardwareMap);
    landingElevator = new Elevator(hardwareMap);

    /*
     * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
     * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
     */

    //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
    //    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

    // OR...  Do Not Activate the Camera Monitor View, to save power
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = "ASPJ+XX/////AAAAGVoUw33gakWRlcLvnWu+vK4/0BincNFIHCnW6Vxig4SR4t7P9G9oRc/LemAnahIp4wXtyWHVoSaLI/7EIpxN/3Tq08Pgs6zsO9pxNukXh8cm7bWcVJ3/RCqAyZReB8kD2duQoOqUlfG9vjVzcLcdu+SFkQP0MkHR8jviuZX30Rp2TWCAtmR/ecjzb6uHPRvkw4IfpWhppDv0TVkCaCvkeMNXI+cR50ythnyVKtZRRs9f0tb3abDhQ1RaFqF9ljyTyzUIgd6M+LnF3fEPJXYvgJSYk4jYyzFf5ATDK93sv8Iem9jNN6zcmKydWFET/vBWhyRLmaU7EkFdseNeWgW5Hgc+G33OeYdPbHy7DKVUB/vg";

    Queue<Leg> travelPath = new LinkedList<>();

    // This is where the travel path is built:
    // Each leg of the trip is added to the queue in this code block.
    // Later, as the opmode runs, the legs are read out and sent to the drive base
    // for execution.
    //
    // mode:     true = turn and drive, false = translate
    // angle:    the desired angle of travel relative to the current bot position and orientation.
    //           in DEGREES
    // distance: the distance to travel in centimeters.
    //
    travelPath.add(new Leg(false, -90, 15));
    //travelPath.add(new Leg(true, 1, 1));
    //travelPath.add(new Leg(true, 1, 1));
    //travelPath.add(new Leg(true, 1, 1));

    // make sure the imu gyro is calibrated before continuing.
    // robot must remain motionless during calibration.
    while (!isStopRequested() && !driveChassis.IMU_IsCalibrated())
    {
      sleep(50);
      idle();
    }

    // Wait for the game to start (driver presses PLAY)
    waitForStart();
    runtime.reset();

    // start to land the bot
    landingElevator.up();
    
    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      // wait for the bot to land (elevator no longer moving)
      while (landingElevator.isMoving());

      // if not driving and there are still legs in the travelPath then send the next leg.
      if(!driveChassis.isMoving() & travelPath.size() != 0 ){
        driveChassis.move( travelPath.remove() );
      }

      IMUTel = driveChassis.drive();

      // Show the elapsed game time.
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addLine().addData("imu status", IMUTel.imuStatus)
          .addData("calib. status", IMUTel.calStatus);
      telemetry.addLine().addData("Z= ", IMUTel.zTheta )
          .addData("Y= ", IMUTel.yTheta )
          .addData("X= ", IMUTel.xTheta );
      telemetry.update();
    }
  }
}
