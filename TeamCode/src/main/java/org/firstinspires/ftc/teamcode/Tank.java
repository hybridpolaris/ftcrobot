package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;


@TeleOp(name="Tank controller")

public class Tank extends LinearOpMode {
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private double drivePower = 1;

    private ShooterController shooterController = new ShooterController(this);
    @Override
    public void runOpMode() {
        telemetry.addData("Status","Init");
        telemetry.update();

        backLeftDrive  = hardwareMap.get(DcMotor.class, "m1");
        backRightDrive = hardwareMap.get(DcMotor.class, "em1");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "m0");
        frontRightDrive = hardwareMap.get(DcMotor.class, "em0");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        shooterController.init();

        waitForStart();
        while (opModeIsActive()) {
            double left_mag = Math.max(0.0000000001,Math.sqrt(gamepad1.left_stick_y*gamepad1.left_stick_y + gamepad1.left_stick_x*gamepad1.left_stick_x));
            double right_mag = Math.max(0.0000000001,Math.sqrt(gamepad1.right_stick_y*gamepad1.right_stick_y + gamepad1.right_stick_x*gamepad1.right_stick_x));
            double left_dir = Math.atan2(gamepad1.left_stick_y/left_mag,gamepad1.left_stick_x/left_mag);
            double right_dir = Math.atan2(gamepad1.right_stick_y/right_mag,gamepad1.right_stick_x/right_mag);
            left_dir = Math.round(left_dir/(Math.PI/2))*(Math.PI/2);
            right_dir = Math.round(right_dir/(Math.PI/2))*(Math.PI/2);
            
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
            
            shooterController.run();

            telemetry.addData("Left drive", left_drive);
            telemetry.addData("Left strafe", left_strafe);
            telemetry.addData("Right drive", right_drive);
            telemetry.addData("Right strafe", right_strafe);
            telemetry.addData("Left mag", left_mag);
            telemetry.addData("Right mag", right_mag);
            telemetry.addData("Right dir", Math.toDegrees(right_dir));
            telemetry.addData("Left dir", Math.toDegrees(left_dir));

            telemetry.addData("Shoot", shooterController.shooterPower);
            telemetry.addData("Transfer", shooterController.transferPower);
            telemetry.addData("Intake", shooterController.intakePower);
            telemetry.update();
        }
    }
}
