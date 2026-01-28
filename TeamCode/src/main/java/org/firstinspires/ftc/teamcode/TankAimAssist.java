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


@TeleOp(name = "Tank controller with aim assist")
//The primary TeleOp mode

public class TankAimAssist extends LinearOpMode {
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
            // Prevents divison by zero if both sticks are idle
            double left_mag = Math.max(0.0000000001, Math.sqrt(gamepad1.left_stick_y * gamepad1.left_stick_y + gamepad1.left_stick_x * gamepad1.left_stick_x));
            double right_mag = Math.max(0.0000000001, Math.sqrt(gamepad1.right_stick_y * gamepad1.right_stick_y + gamepad1.right_stick_x * gamepad1.right_stick_x));
            double left_dir = Math.atan2(gamepad1.left_stick_y / left_mag, gamepad1.left_stick_x / left_mag);
            double right_dir = Math.atan2(gamepad1.right_stick_y / right_mag, gamepad1.right_stick_x / right_mag);
            left_dir = Math.round(left_dir / (Math.PI / 2)) * (Math.PI / 2);
            right_dir = Math.round(right_dir / (Math.PI / 2)) * (Math.PI / 2);
            // Round the direction of both joysticks to multiples of 90 degrees for ease of control
            
            // Drive = joystick Y, Strafe = joystick X
            double left_drive = -Math.sin(left_dir) * left_mag;
            double left_strafe = Math.cos(left_dir) * left_mag;
            double right_drive = -Math.sin(right_dir) * right_mag;
            double right_strafe = Math.cos(right_dir) * right_mag;

            double backLeftPower;
            double backRightPower;

            double frontLeftPower;
            double frontRightPower;

            backLeftPower = left_drive - left_strafe;
            frontLeftPower = left_drive + left_strafe;

            backRightPower = right_drive + right_strafe;
            frontRightPower = right_drive - right_strafe;

            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);

            backRightPower = Range.clip(backRightPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

            backLeftMotor.setPower(backLeftPower * drivePower);
            frontLeftMotor.setPower(frontLeftPower * drivePower);

            backRightMotor.setPower(backRightPower * drivePower);
            frontRightMotor.setPower(frontRightPower * drivePower);
            
            if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up && !gamepad1.y) {
                // Dpad override, uses code from joystick for diagonal movement. Not called if no gamepad inputs are made.
                double x_control = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
                double y_control = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);

                double driveAngle = Math.atan2(x_control, y_control);
                double driveMagnitude = Math.sqrt(Math.pow(x_control, 2) + Math.pow(y_control, 2));

                chassisController.run(driveAngle, driveMagnitude, 0);
            }
            aimAssist.track(gamepad1.y);
            dpsEstimate = Math.round(aimAssist.distance) + 300;
            if (gamepad1.y){
                shooterController.shooterDps = dpsEstimate;
            }
            shooterController.run();
        

            // Tracks shooter variables and motor speed. Also shows which mode is selected(power or degrees/s)
            telemetry.addData("Shooter d/s (A/B)", String.valueOf(shooterController.shooterDps)+" degrees/s");
            telemetry.addData("Shooter velocity left", String.valueOf(Math.round(shooterController.shooterLeftVelocity))+" degrees/s");
            telemetry.addData("Shooter velocity right", String.valueOf(Math.round(shooterController.shooterRightVelocity))+" degrees/s");
            telemetry.addData("Shooter power left", FoxUtil.toPercentage(shooterController.shooterLeftPower));
            telemetry.addData("Shooter power right", FoxUtil.toPercentage(shooterController.shooterRightPower));

            telemetry.addData("Target", aimAssist.lostNavigation?"lost":"available, press Y to track");
            if (aimAssist.lostNavigation){
                telemetry.addData("Distance from target", Math.round(aimAssist.distance));
                telemetry.addData("Power estimate", dpsEstimate);
            }

            telemetry.update();
        }
    }
}