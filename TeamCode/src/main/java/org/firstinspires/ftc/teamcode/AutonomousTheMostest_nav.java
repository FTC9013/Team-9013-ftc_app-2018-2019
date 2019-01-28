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
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name = "AutonomousTheMostest_nav", group = "Linear Opmode")

//@Disabled
public class AutonomousTheMostest_nav extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassisAutonomousIMU driveChassis;
  private IMUTelemetry IMUTel;
  private Elevator landingElevator;
  
  private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private ElapsedTime watchdog = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  

  private static final float mmPerInch = 25.4f;
  private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  private static final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

  OpenGLMatrix lastLocation = null;
  private boolean targetVisible = false;

  private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
  private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
  private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

  private double watchdogTime = 5.0;
  
  private enum goldPosition { UNKNOWN, LEFT, CENTER, RIGHT, TARGETED, MOVED, LOST }
  private enum look { CENTER, LEFT, RIGHT }
  private goldPosition PositionOfTheGoldIs = goldPosition.UNKNOWN;
  private look looking = look.CENTER;

  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;
  WebcamName webcamName;


  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomousIMU(hardwareMap);
    landingElevator = new Elevator(hardwareMap);
    webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

    // build all the drive plans for drive by distance (to move the gold mineral)
    //
    // Each leg of the trip is added to the queue in this code block.
    // As the opmode runs, the queue sent to the drive base for execution.
    //
    // mode:     {FORWARD, BACKWARDS, LEFT, RIGHT, TURN_DRIVE}
    // speed:    the drive speed from 0-100%
    // angle:    the desired angle of travel relative to the ZERO orientation in DEGREES
    //           ZERO is where the bot was facing when the IMU calibrated.  CCW is positive
    // distance: the distance to travel in inches

    Queue<Leg> littleBump = new LinkedList<>();
    littleBump.add(new Leg(Leg.Mode.FORWARD, 20, 0, 0.5));

    Queue<Leg> leftPath = new LinkedList<>();
    leftPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    leftPath.add(new Leg(Leg.Mode.LEFT, 40, 0, 3.5));
    leftPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 20));
    leftPath.add(new Leg(Leg.Mode.LEFT, 40, 0, 13));
    leftPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 10));
    leftPath.add(new Leg(Leg.Mode.BACKWARDS, 40, 0, 15));
    leftPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 40, 0));

    Queue<Leg> centerPath = new LinkedList<>();
    centerPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    centerPath.add(new Leg(Leg.Mode.LEFT, 40, 0, 3.5));
    centerPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 20));
    centerPath.add(new Leg(Leg.Mode.RIGHT, 40, 0, 3.5));
    centerPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 10));
    centerPath.add(new Leg(Leg.Mode.BACKWARDS, 40, 0, 15));
    centerPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 40, 0));

    Queue<Leg> rightPath = new LinkedList<>();
    rightPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    rightPath.add(new Leg(Leg.Mode.LEFT, 40, 0, 3.5));
    rightPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 20));
    rightPath.add(new Leg(Leg.Mode.RIGHT, 40, 0, 20));
    rightPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 10));
    rightPath.add(new Leg(Leg.Mode.BACKWARDS, 40, 0, 15));
    rightPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 40, 0));

    Queue<Leg> lostPath = new LinkedList<>();
    lostPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 0, 0));
    lostPath.add(new Leg(Leg.Mode.FORWARD, 40, 0, 15));
    lostPath.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 40, 0));

    Queue<Leg> lookLeft = new LinkedList<>();
    lookLeft.add(new Leg(Leg.Mode.LEFT, 40, 0, 3.5));
    lookLeft.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 5, 0));

    Queue<Leg> lookRight = new LinkedList<>();
    lookRight.add(new Leg(Leg.Mode.TURN_DRIVE, 40, 355, 0));

    initVuforia();
    
    initTfod();

    /*
     * Load the data sets that for the trackable objects we wish to track. */
    VuforiaTrackables targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");

    VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
    blueRover.setName("Blue-Rover");
    VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
    redFootprint.setName("Red-Footprint");
    VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
    frontCraters.setName("Front-Craters");
    VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
    backSpace.setName("Back-Space");

    /* For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsRoverRuckus);

    /*
     * In order for localization to work, we need to tell the system where each target is on the field, and
     * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
     * Transformation matrices are a central, important concept in the math here involved in localization.
     * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
     * for detailed information. Commonly, you'll encounter transformation matrices as instances
     * of the {@link OpenGLMatrix} class.
     *
     * If you are standing in the Red Alliance Station looking towards the center of the field,
     *     - The X axis runs from your left to the right. (positive from the center to the right)
     *     - The Y axis runs from the Red Alliance Station towards the other side of the field
     *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
     *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
     *
     * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
     *
     * Before being transformed, each target image is conceptually located at the origin of the field's
     *  coordinate system (the center of the field), facing up.
     */

    /*
     * To place the BlueRover target in the middle of the blue perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Then, we translate it along the Y axis to the blue perimeter wall.
     */
    OpenGLMatrix blueRoverLocationOnField =
      OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
                                                    0, 0));
    blueRover.setLocation(blueRoverLocationOnField);

    /*
     * To place the RedFootprint target in the middle of the red perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the negative Y axis to the red perimeter wall.
     */
    OpenGLMatrix redFootprintLocationOnField =
      OpenGLMatrix.translation(0, -mmFTCFieldWidth, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
                                                       0, 180));
    redFootprint.setLocation(redFootprintLocationOnField);

    /*
     * To place the FrontCraters target in the middle of the front perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the negative X axis to the front perimeter wall.
     */
    OpenGLMatrix frontCratersLocationOnField =
      OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
                                                       0, 90));
    frontCraters.setLocation(frontCratersLocationOnField);

    /*
     * To place the BackSpace target in the middle of the back perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the X axis to the back perimeter wall.
     */
    OpenGLMatrix backSpaceLocationOnField =
      OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight)
        .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
                                                    0, -90));
    backSpace.setLocation(backSpaceLocationOnField);

    /*
     * Create a transformation matrix describing where the phone is on the robot.
     *
     * The coordinate frame for the robot looks the same as the field.
     * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
     * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
     *
     * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
     * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
     * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
     *
     * If using the rear (High Res) camera:
     * We need to rotate the camera around it's long axis to bring the rear camera forward.
     * This requires a negative 90 degree rotation on the Y axis
     *
     * If using the Front (Low Res) camera
     * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
     * This requires a Positive 90 degree rotation on the Y axis
     *
     * Next, translate the camera lens to where it is on the robot.
     * In this example, it is centered (left to right),
     * but 110 mm forward of the middle of the robot, and 200 mm above ground level.
     */

    final int CAMERA_FORWARD_DISPLACEMENT = 0;   // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT = 0;   // eg: Camera is ON the robot's center line

    OpenGLMatrix cameraLocationOnRobot =
      OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
        CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(
        EXTRINSIC, YZX, DEGREES, -90, 0, 0));

    /*  Let all the trackable listeners know where the camera is.  */
    for (VuforiaTrackable trackable : allTrackables) {
      ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(
        webcamName, cameraLocationOnRobot);
    }

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
    driveChassis.move(littleBump);
    tfod.activate();
    watchdog.reset();
    
    while (opModeIsActive())
    {
      if (PositionOfTheGoldIs == goldPosition.UNKNOWN) {
        // getUpdatedRecognitions() will return null if no new information is available since
        // the last time that call was made.
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null)
        {
          telemetry.addData("# Object Detected", updatedRecognitions.size());
          if (updatedRecognitions.size() == 3) {
            int goldMineralX = -1;
            int silverMineral1X = -1;
            int silverMineral2X = -1;
            for (Recognition recognition : updatedRecognitions) {
              if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                goldMineralX = (int) recognition.getLeft();
              } else if (silverMineral1X == -1) {
                silverMineral1X = (int) recognition.getLeft();
              } else {
                silverMineral2X = (int) recognition.getLeft();
              }
            }
            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
              if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                telemetry.addData("Gold Mineral Position", "Left");
                PositionOfTheGoldIs = goldPosition.LEFT;
              } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                telemetry.addData("Gold Mineral Position", "Right");
                PositionOfTheGoldIs = goldPosition.RIGHT;
              } else {
                telemetry.addData("Gold Mineral Position", "Center");
                PositionOfTheGoldIs = goldPosition.CENTER;
              }
            }
          }
          telemetry.update();
        }
      }
      else if(PositionOfTheGoldIs == goldPosition.LEFT)
      {
        driveChassis.move(leftPath);
        PositionOfTheGoldIs = goldPosition.TARGETED;
      }
      else if(PositionOfTheGoldIs == goldPosition.CENTER)
      {
        driveChassis.move(centerPath);
        PositionOfTheGoldIs = goldPosition.TARGETED;
      }
      else if(PositionOfTheGoldIs == goldPosition.RIGHT)
      {
        driveChassis.move(rightPath);
        PositionOfTheGoldIs = goldPosition.TARGETED;
      }
      else if(PositionOfTheGoldIs == goldPosition.TARGETED)
      {
        PositionOfTheGoldIs = goldPosition.MOVED;
      }

      // Watchdog timer if no minerals detected for watchdog seconds
      if(watchdog.time() > watchdogTime && PositionOfTheGoldIs == goldPosition.UNKNOWN)
      {
        if(looking == look.CENTER)
        {
          // shuttle left to unhook then look left to to give another look
          driveChassis.move(lookLeft);
          looking = look.LEFT;
        }
        // end search to the left for two seconds without finding.
        else if(looking == look.LEFT && watchdog.time() > watchdogTime + 2 )
        {
          driveChassis.move(lookRight);
          looking = look.RIGHT;
        }
        // end search to the right for two seconds without finding.
        else if(looking == look.RIGHT && watchdog.time() > watchdogTime + 4 )
        {
          driveChassis.move(lostPath);
          // give up, no gold...
          PositionOfTheGoldIs = goldPosition.LOST;
        }
      }

      //  ViewMark navigation here...
      if(PositionOfTheGoldIs == goldPosition.MOVED || PositionOfTheGoldIs == goldPosition.LOST )
      {
        tfod.shutdown();
        // Start tracking the VuMarks
        targetsRoverRuckus.activate();
        landingElevator.down();

        // do all the trackables and claiming of depot stuff until the end of the opmode.
        while (opModeIsActive())
        {
          // check all the trackable target to see which one (if any) is visible.
          targetVisible = false;

          for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
              telemetry.addData("Visible Target", trackable.getName());
              targetVisible = true;

              // getUpdatedRobotLocation() will return null if no new information is available since
              // the last time that call was made, or if the trackable is not currently visible.
              OpenGLMatrix robotLocationTransform = (
                  (VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
              if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
              }
              break; // break if something is visible
            }
          }
          // Provide feedback as to where the robot is located (if we know).
          if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                translation.get(0) / mmPerInch, translation.get(1)
                                                    / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);




            // what to do when a target is found here...




          }
          else {





            // Search for unseen target here!





            telemetry.addData("Visible Target", "none");
          }
          telemetry.update();
        }
      }
    }
  }
  /*
   * Initialize the Vuforia localization engine.
   */
  private void initVuforia() {
    /*
     * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
     * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

//    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
  
  
    parameters.vuforiaLicenseKey = "AQRzHg//////AAABmXMVtox6l0XGn+SvzgNNpWFjA9hRfHwyWN6qA9I+JGvGwQmXG4N89mTxwKDB6dq8QOvsj7xtdR/8l4x+//QG8Ne0A7zdNk9spYVAJqNKWteFOkPYOtlsaVUF0zCQjIRkcMx+iYnNfOIFczN6a41rV3M4cM59tnp59ia8EwGB+P3Sim3UnouhbEfQmy1taJKHSpqRQpeqXJyEvEldrGcJC/UkNvAA42lzNIjusSN70FzpfZUwyf9CSL6TymIfuca35I75wEd9fypv0FhaqMzYM9JqqFGUEULdbruotFc8Ps2KDNrjZO1E+bFyxxlWyfKkS0DwuCYPSmG4+yo2FA7ZVwdF3gEgAx9DjtpD9lWNbg9k";
    parameters.cameraName = webcamName;
    parameters.useExtendedTracking = false;

    //  Instantiate the Vuforia engine
    vuforia = ClassFactory.getInstance().createVuforia(parameters);
  }

  /*
   * Initialize the Tensor Flow Object Detection engine.
   */
  private void initTfod() {
    int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    // init with monitor scree
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

    // init with no monitor screen
 //   TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();

    // set the minimumConfidence to a higher percentage to be more selective when identifying objects.
//    tfodParameters.minimumConfidence = 0.45;

    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
  }
}
