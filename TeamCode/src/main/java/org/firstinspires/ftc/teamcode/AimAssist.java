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

public class AimAssist {
    public boolean lostNavigation = false;
    public double distance = 0;

    private ChassisController chassisController;
    private SimplyPID pid = new SimplyPID(0,0.05,0,0.3);

    AprilTagProcessor aprilTagProcessor;
    Position cameraPosition;
    YawPitchRollAngles cameraOrientation;

    VectorF robotPosition = new VectorF(0,0);
    VectorF tagPosition = new VectorF(0,0);
    VectorF tagOffset = new VectorF(10,10);
    double currentYaw = 0;
    VisionPortal visionPortal;
    LinearOpMode opMode;
    
    private boolean last_a_button = false;
    private boolean redTeam = true;

    public AimAssist(ChassisController controller, LinearOpMode op) {
        chassisController = controller;
        opMode = op;
    }
    public void init() {
        pid.setOuputLimits(-1, 1);
        cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        initAprilTag();
        
        while(opMode.opModeInInit()){
            if (!last_a_button && opMode.gamepad1.a) {
                redTeam = !redTeam;
            }
            last_a_button = opMode.gamepad1.a;
            opMode.telemetry.addData("Team", redTeam ? "red" : "blue");
            opMode.telemetry.addData("Controls", "A to change team");
            opMode.telemetry.update();
        }
        tagOffset.put(1, redTeam?10:-10);
    }

    public void track(boolean move) {
        getPoseEstimation(redTeam?24:20);
        if (lostNavigation || !move) return;
        VectorF relativeTagPosition = tagPosition.added(tagOffset).subtracted(robotPosition);
        double targetAngle = Math.toDegrees(Math.atan2(relativeTagPosition.get(0), relativeTagPosition.get(1)));
        double turnPower = pid.getOutput(System.currentTimeMillis(), targetAngle - currentYaw);
        chassisController.run(0, 0, turnPower);
    }

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
        visionPortalBuilder.setCamera(opMode.hardwareMap.get(WebcamName.class, "camera"));
        visionPortalBuilder.addProcessor(aprilTagProcessor);
        visionPortal = visionPortalBuilder.build();
        
        opMode.telemetry.addData("Camera","Waiting for stream. Press X to cancel");
        opMode.telemetry.update();

        while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) && !opMode.gamepad1.x) {}

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
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
    }

    private void getPoseEstimation(int targetTag) {
        lostNavigation = true;

        List < AprilTagDetection > detectedAprilTags;

        detectedAprilTags = aprilTagProcessor.getDetections();
        for (AprilTagDetection aprilTag: detectedAprilTags) {
            if (aprilTag.metadata != null) {
                if (aprilTag.id == targetTag) {
                    lostNavigation = false;
                    robotPosition.put(0, FoxUtil.toFloat(aprilTag.robotPose.getPosition().x));
                    robotPosition.put(1, FoxUtil.toFloat(aprilTag.robotPose.getPosition().y));
                    tagPosition.put(0, aprilTag.metadata.fieldPosition.get(0));
                    tagPosition.put(1, aprilTag.metadata.fieldPosition.get(1));
                    distance = aprilTag.ftcPose.range;
                    currentYaw = aprilTag.robotPose.getOrientation().getYaw();
                }
            }
        }
    }
}