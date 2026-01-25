package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
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

@Config
@TeleOp(name = "Auto test")
public class Auto extends LinearOpMode {
  public enum Team {RED, BLUE}
  public static Team team = Team.RED;
  private boolean redTeam = team == Team.RED;
  private boolean last_a_button = false;
  private ChassisController chassisController = new ChassisController(this);

  private double turnRate = 90;
  private double moveSpeed = 5;
  // In degrees per second, inch per second
  
  AprilTagProcessor myAprilTagProcessor;
  Position cameraPosition;
  YawPitchRollAngles cameraOrientation;
  double robotAngle = 0;
  Vector robotPosition = new Vector();
  VisionPortal myVisionPortal;

  @Override
  public void runOpMode() {
    // Variables to store the position and orientation of the camera on the robot. Setting these
    // values requires a definition of the axes of the camera and robot:
    // Camera axes:
    // Origin location: Center of the lens
    // Axes orientation: +x right, +y down, +z forward (from camera's perspective)
    // Robot axes (this is typical, but you can define this however you want):
    // Origin location: Center of the robot at field height
    // Axes orientation: +x right, +y forward, +z upward
    // Position:
    // If all values are zero (no translation), that implies the camera is at the center of the
    // robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
    // inches above the ground - you would need to set the position to (-5, 7, 12).
    // Orientation:
    // If all values are zero (no rotation), that implies the camera is pointing straight up. In
    // most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
    // the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
    // it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
    // to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
    cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    chassisController.init();
    // Wait for the match to begin.
    while (opModeInInit()) {
      if (!last_a_button && gamepad1.a) {
        redTeam = !redTeam;
      }
      last_a_button = gamepad1.a;

      telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
      telemetry.addData(">", "Touch START to start OpMode");
      telemetry.addData("Team", redTeam ? "red" : "blue");
      telemetry.update();
    }
    waitForStart();
    while (opModeIsActive()) {
      getPoseEstimation();
      double targetAngle = 0;
      Vector targetPosition = new Vector(0,0);
      
      if (gamepad1.a){
        telemetry.addData("Auto", "Getting on target");
        chassisController.run(0, 0, Math.min(FoxUtil.angleDiff(robotAngle, targetAngle) / turnRate, 1));
      }
      telemetry.update();

      double x_control = gamepad1.left_stick_x + ((gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0));
      double y_control = -gamepad1.left_stick_y + ((gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0));
      double turn = gamepad1.right_stick_x;

      double driveAngle = Math.atan2(x_control, y_control);
      double driveMagnitude = Math.sqrt(Math.pow(x_control, 2) + Math.pow(y_control, 2));

      chassisController.run(driveAngle, driveMagnitude, turn);
    }
  }

  
  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;
    AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();

    libraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());


    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
    myAprilTagProcessorBuilder.setTagLibrary(libraryBuilder.build());

    myAprilTagProcessor = myAprilTagProcessorBuilder.build();

    myVisionPortalBuilder = new VisionPortal.Builder();
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    myVisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void getPoseEstimation() {
    List <AprilTagDetection> detectedAprilTags;

    detectedAprilTags = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(detectedAprilTags));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection aprilTag: detectedAprilTags) {
      // Display info about the detection.
      telemetry.addLine("");
      if (aprilTag.metadata != null) {
        telemetry.addLine("==== (ID " + aprilTag.id + ") " + aprilTag.metadata.name);
        // Only use tags that don't have Obelisk in them since Obelisk tags don't have valid location data
        if (!contains(aprilTag.metadata.name, "Obelisk")) {
          robotAngle = aprilTag.robotPose.getOrientation().getYaw();
          robotPosition = new Vector(aprilTag.robotPose.getPosition().x,aprilTag.robotPose.getPosition().y);

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