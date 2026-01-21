package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;
import java.util.Dictionary;



public class ChassisController {
    private LinearOpMode opMode;
    private double drivePower = 1;
    private double turnWeight = 1;

    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    public double backLeftPower;
    public double backRightPower;
    public double frontLeftPower;
    public double frontRightPower;
    public ChassisController(LinearOpMode _opMode) {
        opMode = _opMode;
    }
    public void init() {
        backLeftDrive  = opMode.hardwareMap.get(DcMotor.class, "m1");
        backRightDrive = opMode.hardwareMap.get(DcMotor.class, "em1");
        frontLeftDrive  = opMode.hardwareMap.get(DcMotor.class, "m0");
        frontRightDrive = opMode.hardwareMap.get(DcMotor.class, "em0");
        
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        opMode.telemetry.addData("Status","Init ChassisControlller module");
    }
    public void run(double moveAngle, double moveMagnitude, double turnPower) {
        if (Double.isNaN(moveAngle)){
            moveAngle = 0;
            moveMagnitude = 0;
        }
        backLeftPower = (Math.cos(moveAngle) - Math.sin(moveAngle)) * moveMagnitude;
        backRightPower = (Math.cos(moveAngle) + Math.sin(moveAngle)) * moveMagnitude;
        frontLeftPower = (Math.cos(moveAngle) - Math.sin(moveAngle)) * moveMagnitude;
        frontRightPower = (Math.cos(moveAngle) + Math.sin(moveAngle)) * moveMagnitude;

        backLeftPower += turnPower;
        frontLeftPower += turnPower;
        backRightPower -= turnPower;
        frontRightPower -= turnPower;

        double magnitude = Math.max(Math.max(Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)), Math.abs(frontLeftPower)), Math.abs(frontRightPower)) / drivePower;

        backLeftPower /= magnitude;
        frontLeftPower /= magnitude;
        backRightPower /= magnitude;
        frontRightPower /= magnitude;

        
        //backLeftDrive.setPower(backLeftPower);
        //frontLeftDrive.setPower(frontLeftPower);

        //backRightDrive.setPower(backRightPower);
        //frontRightDrive.setPower(frontRightPower);
    }
}