package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Util;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;


@TeleOp(name = "Joystick controller with aim assist")
//The primary TeleOp mode

public class JoystickAimAssist extends LinearOpMode {
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private double drivePower = 1;
    private double dpsEstimate = 0;

    private ShooterController shooterController = new ShooterController(this);
    private ChassisController chassisController = new ChassisController(this);
    private AimAssist aimAssist = new AimAssist(chassisController, this);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Init");
        telemetry.update();

        backLeftMotor = hardwareMap.get(DcMotor.class, "m1");
        backRightMotor = hardwareMap.get(DcMotor.class, "em1");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "m0");
        frontRightMotor = hardwareMap.get(DcMotor.class, "em0");

        shooterController.init();
        chassisController.init();
        aimAssist.init();
        //Initiates controllers. Drive motor configurations are done by the chassisController and are not repeated here.

        waitForStart();
        while (opModeIsActive()) {
            double x_control = gamepad1.left_stick_x + ((gamepad1.dpad_right?1:0)-(gamepad1.dpad_left?1:0));
            double y_control = -gamepad1.left_stick_y + ((gamepad1.dpad_up?1:0)-(gamepad1.dpad_down?1:0));
            double turn  =  gamepad1.right_stick_x;

            double driveAngle = Math.atan2(x_control, y_control);
            double driveMagnitude = Math.sqrt(Math.pow(x_control,2) + Math.pow(y_control,2));

            shooterController.run();
            chassisController.run(driveAngle, driveMagnitude, turn);
            
            aimAssist.track(gamepad1.y);
            dpsEstimate = Math.round(aimAssist.distance) + 250;
            if (gamepad1.y){
                shooterController.shooterDps = dpsEstimate;
            }
            shooterController.run();
        

            // Tracks shooter variables and motor speed. Also shows which mode is selected(power or degrees/s)
            telemetry.addData("Shooter d/s (A/B)", String.valueOf(shooterController.shooterDps)+" degrees/s");
            telemetry.addData("","");
            telemetry.addData("Shooter velocity left", String.valueOf(Math.round(shooterController.shooterLeftVelocity))+" degrees/s");
            telemetry.addData("Shooter velocity right", String.valueOf(Math.round(shooterController.shooterRightVelocity))+" degrees/s");
            telemetry.addData("Shooter power left", FoxUtil.toPercentage(shooterController.shooterLeftPower));
            telemetry.addData("Shooter power right", FoxUtil.toPercentage(shooterController.shooterRightPower));
            telemetry.addData("","");
            telemetry.addData("Uses encoder", shooterController.encoder);
            telemetry.addData("Uses power", shooterController.usePower);
            telemetry.addData("Left PIDF", String.valueOf(shooterController.leftCoefficient.p) + " " + String.valueOf(shooterController.leftCoefficient.i) + " " + String.valueOf(shooterController.leftCoefficient.d) + " " + String.valueOf(shooterController.leftCoefficient.f));
            telemetry.addData("Right PIDF", String.valueOf(shooterController.rightCoefficient.p) + " " + String.valueOf(shooterController.rightCoefficient.i) + " " + String.valueOf(shooterController.rightCoefficient.d) + " " + String.valueOf(shooterController.rightCoefficient.f));

            telemetry.addData("","");

            telemetry.addData("Target", aimAssist.lostNavigation?"lost":"available, press Y to track");
            if (!aimAssist.lostNavigation){
                telemetry.addData("Distance from target", Math.round(aimAssist.distance));
                telemetry.addData("Power estimate", dpsEstimate);
            }

            telemetry.update();
        }
    }
} 
