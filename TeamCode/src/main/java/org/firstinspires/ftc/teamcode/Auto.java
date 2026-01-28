package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Auto test")
public class Auto extends LinearOpMode {
  public enum Team {
    RED,
    BLUE
  }
  public static Team team = Team.RED;
  private boolean redTeam = team == Team.RED;
  private boolean last_a_button = false;
  private ChassisController chassisController = new ChassisController(this);

  private double turnRate = 189;
  private double moveSpeed = 5;
  // In degrees per second, inch per second

  AprilTagProcessor aprilTagProcessor;
  Position cameraPosition;
  YawPitchRollAngles cameraOrientation;
  
  double robotAngle = 0;
  double lastError = 0;
  double averageError = 0;
  int errorCount = 0;
  boolean lostNavigation = false;

  VectorF relativeTagPosition = new VectorF(0, 0);
  VectorF robotPosition = new VectorF(0, 0);
  VisionPortal visionPortal;
  // X is forward, Y is sideways for some reason, okay ig

  @Override
  public void runOpMode() {
    cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    chassisController.init();

    SimplyPID pid = new SimplyPID(0,0.05,0,0.3);
    pid.setOuputLimits(-1,1);
    // Wait for the match to begin.
    while (opModeInInit()) {
      if (!last_a_button && gamepad1.a) {
        redTeam = !redTeam;
      }
      last_a_button = gamepad1.a;

      telemetry.addData("Team", redTeam ? "red" : "blue");
      telemetry.update();
    }
    waitForStart();

    double lastTurnPower = 1;
    while (opModeIsActive()) {
      getPoseEstimation();
      if (lostNavigation && gamepad1.a){
        chassisController.run(0, 0, lastTurnPower > 0? -1:1);
        telemetry.addData("Auto", "Trying to regain visual");
        telemetry.update();
        continue;
      }
      double targetAngle = 0;
      VectorF targetPosition = new VectorF(0, 0);

      double turnDelta = FoxUtil.angleDiff(robotAngle, targetAngle);
      telemetry.addData("Direction", robotAngle);
      telemetry.addData("X", robotPosition.getData()[0]);
      telemetry.addData("Y", robotPosition.getData()[1]);
      
      telemetry.addData("TurnRate", turnRate);
      telemetry.addData("Diff", turnDelta);
      
      telemetry.addData("LastError", lastError);
      telemetry.addData("Average error", averageError);
      telemetry.addData("ErrorCount", errorCount);
      
      telemetry.addData("Tag x", relativeTagPosition.getData()[0]);
      telemetry.addData("Tag y", relativeTagPosition.getData()[1]);
      
      
      double angleDelta = -Math.toDegrees(Math.atan2(relativeTagPosition.getData()[0],relativeTagPosition.getData()[1]));
      telemetry.addData("Angle delta", angleDelta);
      if (gamepad1.a) {
        telemetry.addData("Auto", "Getting on target");
        
        double turnPower = pid.getOutput(System.currentTimeMillis(), angleDelta);
        lastTurnPower = turnPower;
        telemetry.addData("TurnPower", turnPower);
        chassisController.run(0, 0, turnPower);

        lastError = pid.lastError;
        averageError = averageError * errorCount + lastError;
        errorCount += 1;
        averageError /= errorCount;

        telemetry.update();
        continue;
      }

      double x_control = gamepad1.left_stick_x + ((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0));
      double y_control = -gamepad1.left_stick_y + ((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
      double turn = gamepad1.right_stick_x;

      double driveAngle = Math.atan2(x_control, y_control);
      double driveMagnitude = Math.sqrt(Math.pow(x_control, 2) + Math.pow(y_control, 2));

      chassisController.run(driveAngle, driveMagnitude, turn);
      lastTurnPower = turn;
      telemetry.update();
    }
  }


  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder aprilTagProcessorBuilder;
    VisionPortal.Builder visionPortalBuilder;
    AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();

    libraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());


    aprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    aprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
    aprilTagProcessorBuilder.setTagLibrary(libraryBuilder.build());

    aprilTagProcessor = aprilTagProcessorBuilder.build();

    visionPortalBuilder = new VisionPortal.Builder();
    visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
    visionPortalBuilder.addProcessor(aprilTagProcessor);
    visionPortal = visionPortalBuilder.build();

    while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
    }
    ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
      exposureControl.setMode(ExposureControl.Mode.Manual);
    }
    exposureControl.setExposure(1, TimeUnit.MILLISECONDS);
    GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
    gainControl.setGain(600);
    
    PtzControl ptzControl = visionPortal.getCameraControl(PtzControl.class);
    ptzControl.setZoom(ptzControl.getMinZoom());
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void getPoseEstimation() {
    lostNavigation = true;

    List < AprilTagDetection > detectedAprilTags;

    detectedAprilTags = aprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(detectedAprilTags));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection aprilTag: detectedAprilTags) {
      // Display info about the detection.
      telemetry.addLine("");
      if (aprilTag.metadata != null) {
        telemetry.addLine("==== (ID " + aprilTag.id + ") " + aprilTag.metadata.name);
        // Only use tags that don't have Obelisk in them since Obelisk tags don't have valid location data
        if (!contains(aprilTag.metadata.name, "Obelisk")) {
          lostNavigation = false;
          robotAngle = -aprilTag.robotPose.getOrientation().getYaw();
          relativeTagPosition = new VectorF(FoxUtil.toFloat(aprilTag.ftcPose.x), FoxUtil.toFloat(aprilTag.ftcPose.y));

          robotPosition = new VectorF(FoxUtil.toFloat(aprilTag.robotPose.getPosition().x), FoxUtil.toFloat(aprilTag.robotPose.getPosition().y));

          telemetry.addLine("XYZ " + JavaUtil.formatNumber(aprilTag.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(aprilTag.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(aprilTag.robotPose.getPosition().z, 6, 1) + "  (inch)");
          telemetry.addLine("PYR " + JavaUtil.formatNumber(aprilTag.robotPose.getOrientation().getPitch(), 6, 1) + " " + JavaUtil.formatNumber(aprilTag.robotPose.getOrientation().getYaw(), 6, 1) + " " + JavaUtil.formatNumber(aprilTag.robotPose.getOrientation().getRoll(), 6, 1) + "  (deg)");
        }
      } else {
        telemetry.addLine("==== (ID " + aprilTag.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(aprilTag.center.x, 6, 0) + "" + JavaUtil.formatNumber(aprilTag.center.y, 6, 0) + " (pixels)");
      }
    }
  }

  /**
   * returns if the containText is inside of the stringToSearch
   */
  private boolean contains(String stringToSearch, String containText) {
    if (stringToSearch.indexOf(containText) + 1 == 0) {
      return false;
    }
    return true;
  }
}