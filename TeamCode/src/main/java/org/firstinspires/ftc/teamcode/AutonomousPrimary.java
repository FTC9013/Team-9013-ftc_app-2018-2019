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


import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name = "AutonomousPrimary", group = "Linear Opmode")

//@Disabled
public class AutonomousPrimary extends LinearOpMode {

  // Declare OpMode members.
  private MecanumDriveChassisAutonomousIMU driveChassis;
  private IMUTelemetry IMUTel;
  private Elevator landingElevator;
  private ElapsedTime runtime = new ElapsedTime();

  private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
  private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
  private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

  private static final float mmPerInch        = 25.4f;
  private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
  private static final float mmTargetHeight   = (6) * mmPerInch; // the height of the center of the target image above the floor

  OpenGLMatrix lastLocation = null;
  private boolean targetVisible = false;

  private VuforiaLocalizer vuforia;
  WebcamName webcamName;

  /**
   * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
   * Detection engine.
   */
  private TFObjectDetector tfod;


  @Override
  public void runOpMode() {

    driveChassis = new MecanumDriveChassisAutonomousIMU(hardwareMap);
    landingElevator = new Elevator(hardwareMap);

    /*
     * Retrieve the camera we are to use.
     */
    webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
    /*
     * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
     * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
     */
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
        "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

    // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
    // OR...  Do Not Activate the Camera Monitor View, to save power
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

    // OR Tensor flow
    TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(cameraMonitorViewId);
    // TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();

    parameters.vuforiaLicenseKey = "AQRzHg//////AAABmXMVtox6l0XGn+SvzgNNpWFjA9hRfHwyWN6qA9I+JGvGwQmXG4N89mTxwKDB6dq8QOvsj7xtdR/8l4x+//QG8Ne0A7zdNk9spYVAJqNKWteFOkPYOtlsaVUF0zCQjIRkcMx+iYnNfOIFczN6a41rV3M4cM59tnp59ia8EwGB+P3Sim3UnouhbEfQmy1taJKHSpqRQpeqXJyEvEldrGcJC/UkNvAA42lzNIjusSN70FzpfZUwyf9CSL6TymIfuca35I75wEd9fypv0FhaqMzYM9JqqFGUEULdbruotFc8Ps2KDNrjZO1E+bFyxxlWyfKkS0DwuCYPSmG4+yo2FA7ZVwdF3gEgAx9DjtpD9lWNbg9k";
    parameters.cameraName = webcamName;
    parameters.useExtendedTracking = false;

    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
    tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);



    /**
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

    /** For convenience, gather together all the trackable objects in one easily-iterable collection */
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    allTrackables.addAll(targetsRoverRuckus);

    /**
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

    /**
     * To place the BlueRover target in the middle of the blue perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Then, we translate it along the Y axis to the blue perimeter wall.
     */
    OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
       .translation(0, mmFTCFieldWidth, mmTargetHeight)
       .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
           0, 0));
    blueRover.setLocation(blueRoverLocationOnField);

    /**
     * To place the RedFootprint target in the middle of the red perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the negative Y axis to the red perimeter wall.
     */
    OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
       .translation(0, -mmFTCFieldWidth, mmTargetHeight)
       .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
           0, 180));
    redFootprint.setLocation(redFootprintLocationOnField);

    /**
     * To place the FrontCraters target in the middle of the front perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the negative X axis to the front perimeter wall.
     */
    OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
       .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
       .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
           0 , 90));
    frontCraters.setLocation(frontCratersLocationOnField);

    /**
     * To place the BackSpace target in the middle of the back perimeter wall:
     * - First we rotate it 90 around the field's X axis to flip it upright.
     * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
     *   and facing inwards to the center of the field.
     * - Then, we translate it along the X axis to the back perimeter wall.
     */
    OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
       .translation(mmFTCFieldWidth, 0, mmTargetHeight)
       .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90,
           0, -90));
    backSpace.setLocation(backSpaceLocationOnField);

    /**
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

    final int CAMERA_FORWARD_DISPLACEMENT  = 0;   // eg: Camera is 110 mm in front of robot center
    final int CAMERA_VERTICAL_DISPLACEMENT = 0;   // eg: Camera is 200 mm above ground
    final int CAMERA_LEFT_DISPLACEMENT     = 0;   // eg: Camera is ON the robot's center line

    OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT,
                CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(
           EXTRINSIC, YZX, DEGREES, -90,0, 0));

    /**  Let all the trackable listeners know where the camera is.  */
    for (VuforiaTrackable trackable : allTrackables)
    {
      ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(
          parameters.cameraName,  cameraLocationOnRobot );
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


    /** Start tracking the data sets we care about. */
    // for trackables.
    // targetsRoverRuckus.activate();

    if (opModeIsActive()) {
      /** Activate Tensor Flow Object Detection. */
      if (tfod != null) {
        tfod.activate();
      }


      while (opModeIsActive()) {

    // Tensor Flow Code  *************
        if (tfod != null) {
          // getUpdatedRecognitions() will return null if no new information is available since
          // the last time that call was made.
          List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
          if (updatedRecognitions != null) {
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
                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                  telemetry.addData("Gold Mineral Position", "Right");
                } else {
                  telemetry.addData("Gold Mineral Position", "Center");
                }
              }
            }
            telemetry.update();
          }
        }
        if (tfod != null) {
          tfod.shutdown();
        }
      }
// TRACKABLES Code **************
//      // check all the trackable target to see which one (if any) is visible.
//      targetVisible = false;
//      for (VuforiaTrackable trackable : allTrackables) {
//        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
//          telemetry.addData("Visible Target", trackable.getName());
//          targetVisible = true;
//
//          // getUpdatedRobotLocation() will return null if no new information is available since
//          // the last time that call was made, or if the trackable is not currently visible.
//          OpenGLMatrix robotLocationTransform = (
//              (VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
//          if (robotLocationTransform != null) {
//            lastLocation = robotLocationTransform;
//          }
//          break;
//        }
//      }
//
//      // Provide feedback as to where the robot is located (if we know).
//      if (targetVisible) {
//        // express position (translation) of robot in inches.
//        VectorF translation = lastLocation.getTranslation();
//        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//            translation.get(0) / mmPerInch, translation.get(1)
//                                                / mmPerInch, translation.get(2) / mmPerInch);
//
//        // express the rotation of the robot in degrees.
//        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
//            rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//      }
//      else {
//        telemetry.addData("Visible Target", "none");
//      }
//      telemetry.update();




//    // start to land the bot
//    landingElevator.up();
//
//    /** Start tracking the data sets we care about. */
//    stonesAndChips.activate();
//
//    // run until the end of the match (driver presses STOP)
//    while (opModeIsActive()) {
//
//      // wait for the bot to land (elevator no longer moving)
//      while (landingElevator.isMoving());
//
//      // if not driving and there are still legs in the travelPath then send the next leg.
//      if(!driveChassis.isMoving() & travelPath.size() != 0 ){
//        driveChassis.move( travelPath.remove() );
//      }
//
//      while (!isStopRequested() && driveChassis.isMoving());
//
//      landingElevator.down();
//
//      IMUTel = driveChassis.drive();
//
//      // Show the elapsed game time.
//      telemetry.addData("Status", "Run Time: " + runtime.toString());
//      telemetry.addLine().addData("imu status", IMUTel.imuStatus)
//          .addData("calib. status", IMUTel.calStatus);
//      telemetry.addLine().addData("Z= ", IMUTel.zTheta )
//          .addData("Y= ", IMUTel.yTheta )
//          .addData("X= ", IMUTel.xTheta );
//      telemetry.update();
    }
  }

  /**
   * A simple utility that extracts positioning information from a transformation matrix
   * and formats it in a form palatable to a human being.
   */
  String format(OpenGLMatrix transformationMatrix) {
    return transformationMatrix.formatAsTransform();
  }
}
