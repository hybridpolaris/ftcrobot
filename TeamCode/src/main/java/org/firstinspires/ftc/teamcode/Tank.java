package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;


@TeleOp(name = "Tank controller")
//The primary TeleOp mode

public class Tank extends LinearOpMode {
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private double drivePower = -1;

    private ShooterController shooterController = new ShooterController(this);
    private ChassisController chassisController = new ChassisController(this);
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Init");
        telemetry.update();

        backLeftDrive = hardwareMap.get(DcMotor.class, "m1");
        backRightDrive = hardwareMap.get(DcMotor.class, "em1");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "m0");
        frontRightDrive = hardwareMap.get(DcMotor.class, "em0");

        shooterController.init();
        chassisController.init();
        //Initiates controllers

        waitForStart();
        while (opModeIsActive()) {
            double left_mag = Math.max(0.0000000001, Math.sqrt(gamepad1.left_stick_y * gamepad1.left_stick_y + gamepad1.left_stick_x * gamepad1.left_stick_x));
            double right_mag = Math.max(0.0000000001, Math.sqrt(gamepad1.right_stick_y * gamepad1.right_stick_y + gamepad1.right_stick_x * gamepad1.right_stick_x));
            double left_dir = Math.atan2(gamepad1.left_stick_y / left_mag, gamepad1.left_stick_x / left_mag);
            double right_dir = Math.atan2(gamepad1.right_stick_y / right_mag, gamepad1.right_stick_x / right_mag);
            left_dir = Math.round(left_dir / (Math.PI / 2)) * (Math.PI / 2);
            right_dir = Math.round(right_dir / (Math.PI / 2)) * (Math.PI / 2);
            //Round input directions to multiples of 90 degrees for ease of control
            
            double left_drive = Math.sin(left_dir) * left_mag;
            double left_strafe = -Math.cos(left_dir) * left_mag;
            double right_drive = Math.sin(right_dir) * right_mag;
            double right_strafe = -Math.cos(right_dir) * right_mag;

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

            backLeftDrive.setPower(backLeftPower * drivePower);
            frontLeftDrive.setPower(frontLeftPower * drivePower);

            backRightDrive.setPower(backRightPower * drivePower);
            frontRightDrive.setPower(frontRightPower * drivePower);

            if (gamepad1.dpad_right || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up) {
                //Dpad override, uses code from joystick for diagonal movement
                double x_control = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
                double y_control = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);

                double driveAngle = Math.atan2(x_control, y_control);
                double driveMagnitude = Math.sqrt(Math.pow(x_control, 2) + Math.pow(y_control, 2));

                chassisController.run(driveAngle, driveMagnitude, 0);
            }

            shooterController.run();

            telemetry.addData("Left drive", left_drive);
            telemetry.addData("Left strafe", left_strafe);
            telemetry.addData("Right drive", right_drive);
            telemetry.addData("Right strafe", right_strafe);
            telemetry.addData("Left mag", left_mag);
            telemetry.addData("Right mag", right_mag);

            telemetry.addData("Shoot", shooterController.shooterPower);
            telemetry.addData("Transfer", shooterController.transferPower);
            telemetry.addData("Intake", shooterController.intakePower);

            telemetry.addData((shooterController.usePower?">":"") + "Shooter strength (A/B)", Util.toPercentage(shooterController.shooterStrength));
            telemetry.addData((shooterController.usePower?"":">") + "Shooter d/s (A/B)", String.valueOf(shooterController.shooterDps)+" degrees/s");
            telemetry.addData("Shooter velocity left", String.valueOf(Math.round(shooterController.shooterLeftVelocity))+" degrees/s");
            telemetry.addData("Shooter velocity right", String.valueOf(Math.round(shooterController.shooterRightVelocity))+" degrees/s");
            telemetry.addData("Shooter power left", Util.toPercentage(shooterController.shooterLeftPower));
            telemetry.addData("Shooter power right", Util.toPercentage(shooterController.shooterRightPower));
            telemetry.update();

            if (gamepad1.x) {
                break; //Killswitch
            }
        }
    }
}