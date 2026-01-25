package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;
import java.util.Dictionary;


/// This class controls the vast majority of chassis behaviour and movement.
/// This and ShooterController used to be part of the opmode themselves, we decided to separate them for a more modular codebase, making it easier to
/// change configs and code autonomous opmodes later down the line. 
public class ChassisController {
    private LinearOpMode opMode;

    // How strong the motors are
    private double drivePower = 1;
    // How much turn interacts with movement inputs if both are pressed. Higher turn weights will take precedent over lateral movement.
    private double turnWeight = 1;

    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;

    public double backLeftPower;
    public double backRightPower;
    public double frontLeftPower;
    public double frontRightPower;
    public static final PIDFCoefficients coefficient = new PIDFCoefficients(0,0,0,0);
    public ChassisController(LinearOpMode _opMode) {
        opMode = _opMode;
    }
    public void init() {
        /// Initial setup for the motors. PIDF will be tuned
        backLeftMotor  = opMode.hardwareMap.get(DcMotorEx.class, "m1");
        backRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "em1");
        frontLeftMotor  = opMode.hardwareMap.get(DcMotorEx.class, "m0");
        frontRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "em0");
        
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        
        ((DcMotorEx) backLeftMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        ((DcMotorEx) backRightMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        ((DcMotorEx) frontLeftMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        ((DcMotorEx) frontRightMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        opMode.telemetry.addData("Status","Init ChassisControlller module");
    }
    public void run(double moveAngle, double moveMagnitude, double turnPower) {
        // Accounts for a NaN move angle as a result of atan(0,0)
        if (Double.isNaN(moveAngle)){
            moveAngle = 0;
            moveMagnitude = 0;
        }
        // Simple trig, I hope
        backLeftPower = (Math.cos(moveAngle) - Math.sin(moveAngle)) * moveMagnitude;
        backRightPower = (Math.cos(moveAngle) + Math.sin(moveAngle)) * moveMagnitude;
        frontLeftPower = (Math.cos(moveAngle) + Math.sin(moveAngle)) * moveMagnitude;
        frontRightPower = (Math.cos(moveAngle) - Math.sin(moveAngle)) * moveMagnitude;

        backLeftPower += turnPower;
        frontLeftPower += turnPower;
        backRightPower -= turnPower;
        frontRightPower -= turnPower;

        // Due to naively applying turn power to motor, this block scales all motor down so that the highest powered motor is 1. Also applies drivePower.
        double magnitude = Math.max(Math.max(Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)), Math.abs(frontLeftPower)), Math.abs(frontRightPower)) / drivePower;

        backLeftPower /= magnitude;
        frontLeftPower /= magnitude;
        backRightPower /= magnitude;
        frontRightPower /= magnitude;

        
        backLeftMotor.setPower(backLeftPower);
        frontLeftMotor.setPower(frontLeftPower);

        backRightMotor.setPower(backRightPower);
        frontRightMotor.setPower(frontRightPower);
    }
}