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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

@Autonomous(name = "AutonomousPrimaryTFlow", group = "Linear Opmode")

//@Disabled
public class AutonomousPrimaryTFlow extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassisAutonomousIMU driveChassis;
  private IMUTelemetry IMUTel;
  private Elevator landingElevator;
  private ElapsedTime runtime = new ElapsedTime();

  private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
  private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
  private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;


  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomousIMU(hardwareMap);
    landingElevator = new Elevator(hardwareMap);

    initVuforia();

    if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
      initTfod();
    } else {
      telemetry.addData("Sorry!", "This device is not compatible with TFOD");
    }


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

    tfod.activate();

    while (opModeIsActive())
    {
      // getUpdatedRecognitions() will return null if no new information is available since
      // the last time that call was made.
      List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
      if (updatedRecognitions != null)
      {
        telemetry.addData("# Object Detected", updatedRecognitions.size());
        if (updatedRecognitions.size() == 3)
        {
          int goldMineralX = -1;
          int silverMineral1X = -1;
          int silverMineral2X = -1;
          for (Recognition recognition : updatedRecognitions)
          {
            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
            {
              goldMineralX = (int) recognition.getLeft();
            }
            else if (silverMineral1X == -1)
            {
              silverMineral1X = (int) recognition.getLeft();
            }
            else
            {
              silverMineral2X = (int) recognition.getLeft();
            }
          }
          if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1)
          {
            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X)
            {
              telemetry.addData("Gold Mineral Position", "Left");
            }
            else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X)
            {
              telemetry.addData("Gold Mineral Position", "Right");
            }
            else
            {
              telemetry.addData("Gold Mineral Position", "Center");
            }
          }
        }
        telemetry.update();
      }
    }
      tfod.shutdown();
  }
  /**
   * Initialize the Vuforia localization engine.
   */
  private void initVuforia() {
    /*
     * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
     */
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    parameters.vuforiaLicenseKey = "AQRzHg//////AAABmXMVtox6l0XGn+SvzgNNpWFjA9hRfHwyWN6qA9I+JGvGwQmXG4N89mTxwKDB6dq8QOvsj7xtdR/8l4x+//QG8Ne0A7zdNk9spYVAJqNKWteFOkPYOtlsaVUF0zCQjIRkcMx+iYnNfOIFczN6a41rV3M4cM59tnp59ia8EwGB+P3Sim3UnouhbEfQmy1taJKHSpqRQpeqXJyEvEldrGcJC/UkNvAA42lzNIjusSN70FzpfZUwyf9CSL6TymIfuca35I75wEd9fypv0FhaqMzYM9JqqFGUEULdbruotFc8Ps2KDNrjZO1E+bFyxxlWyfKkS0DwuCYPSmG4+yo2FA7ZVwdF3gEgAx9DjtpD9lWNbg9k";
    parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
  }

  /**
   * Initialize the Tensor Flow Object Detection engine.
   */
  private void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
  }
}
