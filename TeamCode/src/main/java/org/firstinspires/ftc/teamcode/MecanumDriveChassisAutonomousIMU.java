package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Queue;

class MecanumDriveChassisAutonomousIMU
{
  private DcMotor leftFrontDrive;
  private DcMotor leftRearDrive;
  private DcMotor rightFrontDrive;
  private DcMotor rightRearDrive;
  private BNO055IMU imu;

  private Orientation angles; // stores the current orientation of the bot from the IMU

  private static double leftFrontDriveSpeed;
  private static double leftRearDriveSpeed;
  private static double rightFrontDriveSpeed;
  private static double rightRearDriveSpeed;
  
  private static IMUTelemetry IMUTel;
  

  // speed for am IMU turn
  private final double MaxTurnSpeed = 0.2;
  private final int countsPerTurnDegree = 15;
  private final double angleError = 1;          // turn angle error cutoff to stop turning.
  private final int countsPerDriveInch = 10000/117;
  private final int countsStrafePerInch = 5000/51;

  MecanumDriveChassisAutonomousIMU(HardwareMap hardwareMap)
  {

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    leftFrontDrive = hardwareMap.get(DcMotor.class, "lFront"); //hub 3 port 0
    leftRearDrive = hardwareMap.get(DcMotor.class, "lRear"); //hub 3 port 2
    rightFrontDrive = hardwareMap.get(DcMotor.class, "rFront"); //hub 3 port 1
    rightRearDrive = hardwareMap.get(DcMotor.class, "rRear"); //hub 3 port 3

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Motors on one side reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    // A positive power number should drive the robot forward regardless of the motor's
    // position on the robot.
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

    // Set all the motor speeds to 0.
    rightFrontDriveSpeed = 0;
    leftFrontDriveSpeed = 0;
    rightRearDriveSpeed = 0;
    leftRearDriveSpeed = 0;

    rightFrontDrive.setPower(rightFrontDriveSpeed);
    leftFrontDrive.setPower(leftFrontDriveSpeed);
    rightRearDrive.setPower(rightRearDriveSpeed);
    leftRearDrive.setPower(leftRearDriveSpeed);

    // reset encoders and target positions to 0
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
    leftFrontDrive.setTargetPosition(0);
    leftRearDrive.setTargetPosition(0);
    rightFrontDrive.setTargetPosition(0);
    rightRearDrive.setTargetPosition(0);

    leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    // Get and initialize the IMU. (we will use the imu on hub id = 3)
    imu = hardwareMap.get(BNO055IMU.class, "imu1");

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    // set the initial imu mode parameters.
    parameters.mode = BNO055IMU.SensorMode.IMU;
    // Set up the parameters with which we will use our IMU. Note that integration
    // algorithm here just reports accelerations to the logcat log; it doesn't actually
    // provide positional information.
    parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = false;
//    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu.initialize(parameters);
  
    IMUTel = new IMUTelemetry();
  }

  boolean IMU_IsCalibrated () {
    return imu.isGyroCalibrated();
  }
  
  /*
   * returns ture if the bot is still moving
   */
  boolean isMoving() {
//    return moving;
    return rightFrontDrive.isBusy() || leftFrontDrive.isBusy()
           || rightRearDrive.isBusy()
           || leftRearDrive.isBusy();
  }
  
  // execute a path (list of drive legs)
  void move(Queue<Leg> path ) {
    Leg currentLeg;
    // as long as there are legs to drive...
    while(path.size() !=0 ){
      currentLeg = path.remove();
      // speed is passed in as a % from 1 to 100.  motor speed control range is 0-1 with
      // 1 being 100%
      setMoveSpeed(currentLeg.speed/100);

      switch (currentLeg.mode) {
        case FORWARD:
          driveForward(currentLeg.distance);
          break;

        case BACKWARDS:
          driveBackwards(currentLeg.distance);
          break;

        case LEFT:
          strafeLeft(currentLeg.distance);
          break;

        case RIGHT:
          strafeRight(currentLeg.distance);
          break;

        case TURN_DRIVE:
          setMoveSpeed(MaxTurnSpeed);
          turnToAbsoluteAngle(currentLeg.angle);
          setMoveSpeed(currentLeg.speed/100);
          driveForward(currentLeg.distance);
          break;
      }
    }
    // set speed back to 0 at the end of the drive
    setMoveSpeed(0);

  }

  void driveForward(double inches){
    // Move forward
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    leftRearDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    rightFrontDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    rightRearDrive.setTargetPosition((int)(inches * countsPerDriveInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }
  
  void driveBackwards(double inches){
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    leftRearDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    rightFrontDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    rightRearDrive.setTargetPosition(-(int)(inches * countsPerDriveInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }
  
  void strafeLeft(double inches){
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    leftRearDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    rightFrontDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    rightRearDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }

  void strafeRight(double inches){
    stopAndResetEncoders();
    leftFrontDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    leftRearDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    rightFrontDrive.setTargetPosition(-(int)(inches * countsStrafePerInch));
    rightRearDrive.setTargetPosition((int)(inches * countsStrafePerInch));
    runToPosition();
    // wait for motors to stop
    while (isMoving());
  }

  void turnToAbsoluteAngle(double desiredAngle){
    double delta;
    do
    {
      // desired angle in degrees +/- 0 to 180 where CCW is + and CW is -
      angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      delta = desiredAngle - angles.firstAngle;
      if(delta > 180 ) {delta -= 360;}
      if(delta < -180 ) {delta += 360;}

      // as long as the bot is not on the desired heading, keep turning
      leftFrontDrive.setTargetPosition((int) (countsPerTurnDegree * -delta));
      leftRearDrive.setTargetPosition((int) (countsPerTurnDegree * -delta));
      rightFrontDrive.setTargetPosition((int) (countsPerTurnDegree * delta));
      rightRearDrive.setTargetPosition((int) (countsPerTurnDegree * delta));
      runToPosition();
      // wait for motors to stop
      while (isMoving()) ;
    } while (Math.abs(delta) > angleError );  // keep going till close enough to desired angle
  }

  void setMoveSpeed(double speed) {
    rightFrontDrive.setPower(speed);
    leftFrontDrive.setPower(speed);
    rightRearDrive.setPower(speed);
    leftRearDrive.setPower(speed);
  }
  
  void stopAndResetEncoders(){
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  
  void runToPosition() {
    leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  
  
}
