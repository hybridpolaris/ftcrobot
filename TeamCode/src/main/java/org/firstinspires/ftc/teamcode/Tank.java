package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;



@TeleOp(name="Tank")

public class Tank extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status","Init");
        telemetry.update();

        backLeftDrive  = hardwareMap.get(DcMotor.class, "m0");
        backRightDrive = hardwareMap.get(DcMotor.class, "m1");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "m2");
        frontRightDrive = hardwareMap.get(DcMotor.class, "m3");

        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);

        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        
        boolean shoot = false;
        double power = 1;

        while (opModeIsActive()) {
            double left_drive = -gamepad1.left_stick_y;
            double left_strafe = gamepad1.left_stick_x;

            double right_drive = -gamepad1.right_stick_y;
            double right_strafe = gamepad1.right_stick_x;

            shoot = gamepad1.a;

            double backLeftPower;
            double backRightPower;

            double frontLeftPower;
            double frontRightPower;

            backLeftPower = left_drive - left_strafe;
            frontLeftPower = left_drive + left_strafe;

            backRightPower = right_drive + right_strafe;
            frontRightPower = right_drive - right_strafe;
            
            backLeftPower    = Range.clip(backLeftPower, -1.0, 1.0) ;
            frontLeftPower    = Range.clip(frontLeftPower, -1.0, 1.0) ;

            backRightPower   = Range.clip(backRightPower, -1.0, 1.0) ;
            frontRightPower   = Range.clip(frontRightPower, -1.0, 1.0) ;
            

            backLeftDrive.setPower(backLeftPower * power);
            frontLeftDrive.setPower(frontLeftPower * power);

            backRightDrive.setPower(backRightPower * power);
            frontRightDrive.setPower(frontRightPower * power);

            telemetry.addData("Shoot", gamepad1.a);
            telemetry.addData("Left drive", left_drive);
            telemetry.addData("Left strafe", left_strafe);
            telemetry.addData("Right drive", right_drive);
            telemetry.addData("Right strafe", right_strafe);

            telemetry.update();
        }
    }
}
