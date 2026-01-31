package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    
    public double backLeftVelocity;
    public double backRightVelocity;
    public double frontLeftVelocity;
    public double frontRightVelocity;

    public boolean useVelocity  = false;
    public double maxVelocity = 500;
    
    public static PIDFCoefficients coefficient = new PIDFCoefficients(0,0,0,0);
    public static double F0 = 0;
    public static double F1 = 0;
    public static double F2 = 0;
    public static double F3 = 0;
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
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        //RUN_WITHOUT_ENCODER
        //RUN_USING_ENCODER
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        ((DcMotorEx) backLeftMotor).setMotorEnable();
        ((DcMotorEx) backRightMotor).setMotorEnable();
        ((DcMotorEx) frontLeftMotor).setMotorEnable();
        ((DcMotorEx) frontRightMotor).setMotorEnable();

        //((DcMotorEx) backLeftMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        //((DcMotorEx) backRightMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        //((DcMotorEx) frontLeftMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        //((DcMotorEx) frontRightMotor).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        opMode.telemetry.addData("Status","Init ChassisControlller module");
    }
    public void revolve(DcMotor motor, double direction){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(((Double)(537.6 * direction)).intValue());
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);
    }
    public void turnBackLeftMotor(double direction, int count){
        revolve(backLeftMotor, direction * count);
    }
    public void turnBackRightMotor(double direction, int count){
        revolve(backRightMotor, direction * count);
    }
    public void turnFrontLeftMotor(double direction, int count){
        revolve(frontLeftMotor, direction * count);
    }
    public void turnFrontRightMotor(double direction, int count){
        revolve(frontRightMotor, direction * count);
    }
    public void move(double moveAngle, int rounds) {
        turnBackLeftMotor((Math.cos(moveAngle) - Math.sin(moveAngle)),rounds);
        turnBackRightMotor((Math.cos(moveAngle) + Math.sin(moveAngle)),rounds);
        turnFrontLeftMotor((Math.cos(moveAngle) + Math.sin(moveAngle)),rounds);
        turnFrontLeftMotor((Math.cos(moveAngle) - Math.sin(moveAngle)),rounds);
    }
    public void setBreak(){
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(double moveAngle, double moveMagnitude, double turnPower) {
        if (turnPower == 0 && moveMagnitude == 0){
            /*backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        }else{
            /*backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);*/
        }
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
        
        double microPower = Math.max(Math.min(turnPower * 2, 1), turnPower);
        double reservePower = turnPower * 2 - microPower; 
        backLeftPower += turnPower;
        frontLeftPower += turnPower;
        backRightPower -= turnPower;
        frontRightPower -= turnPower;

        // Due to naively applying turn power to motor, this block scales all motor down so that the highest powered motor is 1. Also applies drivePower.
        double magnitude = Math.max(Math.max(Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)), Math.abs(frontLeftPower)), Math.abs(frontRightPower)) / drivePower / (Math.abs(moveMagnitude) + Math.abs(turnPower));
        
        backLeftPower /= magnitude;
        frontLeftPower /= magnitude;
        backRightPower /= magnitude;
        frontRightPower /= magnitude;
        
        

        if (useVelocity){
            ((DcMotorEx) backLeftMotor).setVelocity(backLeftPower * maxVelocity, AngleUnit.DEGREES);
            ((DcMotorEx) frontLeftMotor).setVelocity(-frontLeftPower * maxVelocity, AngleUnit.DEGREES);
            ((DcMotorEx) backRightMotor).setVelocity(backRightPower * maxVelocity, AngleUnit.DEGREES);
            ((DcMotorEx) frontRightMotor).setVelocity(-frontRightPower  * maxVelocity, AngleUnit.DEGREES);
        }else {
            backLeftMotor.setPower(backLeftPower);
            frontLeftMotor.setPower(frontLeftPower);
            backRightMotor.setPower(backRightPower);
            frontRightMotor.setPower(frontRightPower);
        }
        
        
        backLeftPower = backLeftMotor.getPower();
        frontLeftPower = frontLeftMotor.getPower();
        backRightPower = backRightMotor.getPower();
        frontRightPower = frontRightMotor.getPower();

        backLeftVelocity = ((DcMotorEx) backLeftMotor).getVelocity(AngleUnit.DEGREES);
        backRightVelocity = ((DcMotorEx) backRightMotor).getVelocity(AngleUnit.DEGREES);
        frontLeftVelocity = ((DcMotorEx) frontLeftMotor).getVelocity(AngleUnit.DEGREES);
        frontRightVelocity = ((DcMotorEx) frontRightMotor).getVelocity(AngleUnit.DEGREES);
    }
}