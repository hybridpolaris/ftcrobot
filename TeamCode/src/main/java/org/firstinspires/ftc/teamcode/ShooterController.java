package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import java.lang.Math;
import java.util.Dictionary;


/// Controls the entirety of the shooting mechanism.
public class ShooterController {
    private LinearOpMode opMode;

    // The motors and servos
    private DcMotor shooterMotorLeft = null;
    private DcMotor shooterMotorRight = null;
    private DcMotor intakeMotor = null;
    private DcMotor transferMotor = null;
    private Servo rightClamp = null;
    private Servo leftClamp = null;

    private double intakeDir = 0;
    private boolean shooter = false;
    
    public boolean revving = false;
    public boolean shooting = false;

    // Debounce, to force a full press of bumpers and triggers
    private boolean last_right_bumper = false;
    private boolean last_left_pair = false;
    private boolean last_a = false;
    private boolean last_b = false;
    private boolean last_y = false;

    // Initial values 
    private double intakeStrength = 1;
    public double shooterStrength = 1;
    public double shooterDps = 300;

    //Whether to use power instead of degrees/s
    public boolean usePower = false;

    // Variables that directly influences the motors
    public double shooterPower;
    public double shooterOutputDps;
    public double intakePower;
    public double transferPower;

    // Variables purely for telemetry and tracking purposes, will be read and displayed by opmodes.
    public double shooterLeftPower;
    public double shooterRightPower;
    public double shooterLeftVelocity;
    public double shooterRightVelocity;

    // The PID coefficients to be tuned.
    public static PIDFCoefficients coefficient = new PIDFCoefficients(20, 1, 0.1, 0.5);

    public ShooterController(LinearOpMode _opMode) {
        opMode = _opMode;
    }
    public void init() {
        // Initial motor configs
        shooterMotorLeft = opMode.hardwareMap.get(DcMotor.class, "m3");
        shooterMotorRight = opMode.hardwareMap.get(DcMotor.class, "em3");
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "em2");
        transferMotor = opMode.hardwareMap.get(DcMotor.class, "m2");
        leftClamp = opMode.hardwareMap.get(Servo.class, "s0");
        rightClamp = opMode.hardwareMap.get(Servo.class, "es0");

        shooterMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterMotorRight.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);

        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        ((DcMotorEx) shooterMotorLeft).setMotorEnable();
        ((DcMotorEx) shooterMotorRight).setMotorEnable();
        
        ((DcMotorEx) shooterMotorLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        ((DcMotorEx) shooterMotorRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);

        leftClamp.setDirection(Servo.Direction.FORWARD);
        rightClamp.setDirection(Servo.Direction.FORWARD);
        leftClamp.scaleRange(0, 1);
        rightClamp.scaleRange(0, 1);

        opMode.telemetry.addData("Status", "Init ShooterController module");
    }
    public void setShooterVelocity(double vel){
        ((DcMotorEx)shooterMotorLeft).setVelocity(vel,AngleUnit.DEGREES);
        ((DcMotorEx)shooterMotorRight).setVelocity(vel,AngleUnit.DEGREES);
    }
    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }
    public void setTransferPower(double power){
        transferMotor.setPower(power);
    }
    public void toggleClamp(boolean open){
        if (open){
            leftClamp.setPosition(0.15);
            rightClamp.setPosition(0.75);
        } else {
            leftClamp.setPosition(0.3);
            rightClamp.setPosition(0.6);
        }
    }
    public void setIdle(){
        toggleClamp(false);
        setTransferPower(-1);
        setIntakePower(1);
        setShooterVelocity(0);
    }
    public void setWeakIdle(){
        toggleClamp(false);
        setTransferPower(-1);
        setIntakePower(0.3);
        setShooterVelocity(0);
    }
    public void setRevving(double vel){
        toggleClamp(true);
        setTransferPower(-1);
        setIntakePower(0.3);
        setShooterVelocity(vel);
    }
    public void setShooting(double vel){
        toggleClamp(true);
        setTransferPower(1);
        setIntakePower(1);
        setShooterVelocity(vel);
    }
    public void stopAll(){
        toggleClamp(false);
        setTransferPower(0);
        setIntakePower(0);
        setShooterVelocity(0);
    }
    public void run() {
        // Adjust the strength of the shooter. Mainly for calibration purposes
        if (opMode.gamepad1.a && !last_a) {
            if (usePower) {
                shooterStrength += 0.05;
            } else {
                shooterDps += 10;
            }
        }
        if (opMode.gamepad1.b && !last_b) {
            if (usePower) {
                shooterStrength -= 0.05;
            } else {
                shooterDps -= 10;
            }
        }
        // Toggles between using power and degrees/s
        if (opMode.gamepad1.y && !last_y) {
            //usePower = !usePower;
        }
        shooterStrength = Range.clip(shooterStrength, 0, 1);
        // Debounce
        last_a = opMode.gamepad1.a;
        last_b = opMode.gamepad1.b;
        last_y = opMode.gamepad1.y;
        // Toggles the intake motor to suck in artifacts with left bumper, or spit them out with left trigger.
        if (last_left_pair != (opMode.gamepad1.left_bumper || (opMode.gamepad1.left_trigger > 0))) {
            if (opMode.gamepad1.left_bumper) {
                intakeDir = intakeDir == 1 ? 0 : 1;
            }
            if ((opMode.gamepad1.left_trigger > 0)) {
                intakeDir = intakeDir == -1 ? 0 : -1;
            }
        }
        // Debounce, so the intake is only toggled on a full press (press and release)
        last_left_pair = opMode.gamepad1.left_bumper || (opMode.gamepad1.left_trigger > 0);

        // Toggles revving. While in revving mode, the shooter starts spinning, the clamps are released, the intake is slowed down and the transfer is spun backwards to stop artifacts from going up.
        if (opMode.gamepad1.right_bumper) {
            if (!last_right_bumper) {
                if (revving) {
                    revving = false;
                } else {
                    revving = true;
                    intakeDir = 1;
                }
            }
        }
        last_right_bumper = opMode.gamepad1.right_bumper;

        // While revving, if right trigger is held, go into shooting mode. In shooting mode, the intake is sped up and transfer drive spun forward to send an artifact to the shooter.
        // This is a hold instead of a toggle to make it easier to shoot artifacts one at a time, maintaining accuracy
        if (opMode.gamepad1.right_trigger > 0 && revving) {
            shooting = true;
        } else {
            shooting = false;
        }

        if (revving && !shooting) {
            // Revving mode
            shooter = true;
            leftClamp.setPosition(0.15);
            rightClamp.setPosition(0.75);
            intakeStrength = 0.3;
            transferPower = -1;
        } else if (shooting) {
            // Shooting mode
            shooter = true;
            leftClamp.setPosition(0.15);
            rightClamp.setPosition(0.75);
            intakeStrength = 0.8;
            transferPower = 1;
        } else {
            // Idle mode
            leftClamp.setPosition(0.33);
            rightClamp.setPosition(0.57);
            intakeStrength = 0.8;
            transferPower = 0;
            shooter = false;
        }

        // Scale the final intake power by the direction.
        intakePower = intakeStrength * intakeDir;
        transferPower = transferPower;

        if (usePower) {
            shooterPower = shooter ? shooterStrength : 0;
            shooterMotorLeft.setPower(shooterPower);
            shooterMotorRight.setPower(shooterPower * 0.955);
        } else {
            shooterOutputDps = shooter ? shooterDps : 0;
            ((DcMotorEx)shooterMotorLeft).setVelocity(shooterOutputDps,AngleUnit.DEGREES);
            ((DcMotorEx)shooterMotorRight).setVelocity(shooterOutputDps,AngleUnit.DEGREES);
        }

        intakeMotor.setPower(intakePower);
        transferMotor.setPower(transferPower);

        // Set tracking variables to be read by opmodes
        shooterLeftVelocity = ((DcMotorEx) shooterMotorLeft).getVelocity(AngleUnit.DEGREES);
        shooterRightVelocity = ((DcMotorEx) shooterMotorRight).getVelocity(AngleUnit.DEGREES);
        shooterLeftPower = shooterMotorLeft.getPower();
        shooterRightPower = shooterMotorRight.getPower();
        coefficient = ((DcMotorEx)shooterMotorRight).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}