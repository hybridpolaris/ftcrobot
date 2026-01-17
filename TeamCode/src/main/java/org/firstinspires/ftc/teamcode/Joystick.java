package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;



@TeleOp(name="Joystick controls")

public class Joystick extends LinearOpMode {
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    private ShooterController shooterController = new ShooterController(this);
    @Override
    public void runOpMode() {
        telemetry.addData("Status","Init");
        telemetry.update();

        backLeftDrive  = hardwareMap.get(DcMotor.class, "m1");
        backRightDrive = hardwareMap.get(DcMotor.class, "em1");
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "m0");
        frontRightDrive = hardwareMap.get(DcMotor.class, "em0");

        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        
        double turn = 0;
        double strafe = 0;
        double drive = 0;
        boolean shoot = false;
        double power = -1;
        double x_control = 0;
        double y_control = 0;
        
        shooterController.init();
        while (opModeIsActive()) {
            x_control = gamepad1.left_stick_x + ((gamepad1.dpad_right?1:0)-(gamepad1.dpad_left?1:0));
            y_control = -gamepad1.left_stick_y + ((gamepad1.dpad_up?1:0)-(gamepad1.dpad_down?1:0));
            drive = y_control;
            turn  =  gamepad1.right_stick_x;
            strafe = x_control;
            shoot = gamepad1.a;

            double backLeftPower;
            double backRightPower;
            double frontLeftPower;
            double frontRightPower;
            double leftTurnOverride = -turn;
            double rightTurnOverride = turn;

            backLeftPower = -drive + strafe;
            backRightPower = -drive - strafe;
            frontLeftPower = drive + strafe;
            frontRightPower = drive - strafe;
            
            backLeftPower    = Range.clip(leftTurnOverride + (backLeftPower) * Math.abs(1-turn), -1.0, 1.0) ;
            backRightPower   = Range.clip(rightTurnOverride + (backRightPower) * Math.abs(1-turn), -1.0, 1.0) ;
            frontLeftPower    = Range.clip(leftTurnOverride - (frontLeftPower) * Math.abs(1-turn), -1.0, 1.0) ;
            frontRightPower   = Range.clip(rightTurnOverride - (frontRightPower) * Math.abs(1-turn), -1.0, 1.0) ;
            

            backLeftDrive.setPower(backLeftPower * power);
            backRightDrive.setPower(backRightPower * power);
            frontLeftDrive.setPower(frontLeftPower * power);
            frontRightDrive.setPower(frontRightPower * power);

            shooterController.run();

            telemetry.addData("Drive", -gamepad1.left_stick_y);
            telemetry.addData("Turn", gamepad1.right_stick_x);
            telemetry.addData("Shoot", gamepad1.a);
            telemetry.update();
        }
    }
}
