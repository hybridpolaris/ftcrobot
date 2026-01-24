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


/// Controls the entirety of the shooting mechanism.
public class ShooterController {
    private LinearOpMode opMode;

    private DcMotor shooterDriveLeft = null;
    private DcMotor shooterDriveRight = null;
    private DcMotor intakeDrive = null;
    private DcMotor transferDrive = null;
    private Servo rightClamp = null;
    private Servo leftClamp = null;

    private double intakeDir = 0;
    private boolean shooter = false;
    public boolean revving = false;
    public boolean shooting = false;

    private boolean last_right_bumper = false;
    private boolean last_left_pair = false;
    private double intakeStrength = 1;

    public double shooterPower;
    public double intakePower;
    public double transferPower;
    public ShooterController(LinearOpMode _opMode) {
        opMode = _opMode;
    }
    public void init() {
        shooterDriveLeft = opMode.hardwareMap.get(DcMotor.class, "m3");
        shooterDriveRight = opMode.hardwareMap.get(DcMotor.class, "em3");
        intakeDrive = opMode.hardwareMap.get(DcMotor.class, "em2");
        transferDrive = opMode.hardwareMap.get(DcMotor.class, "m2");
        leftClamp = opMode.hardwareMap.get(Servo.class, "s0");
        rightClamp = opMode.hardwareMap.get(Servo.class, "es0");

        shooterDriveLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterDriveRight.setDirection(DcMotor.Direction.REVERSE);
        intakeDrive.setDirection(DcMotor.Direction.FORWARD);
        transferDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftClamp.setDirection(Servo.Direction.FORWARD);
        rightClamp.setDirection(Servo.Direction.FORWARD);
        leftClamp.scaleRange(0,1);
        rightClamp.scaleRange(0,1);
        
        opMode.telemetry.addData("Status","Init ShooterController module");
    }
    public void run() {
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
                if (revving){
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
        if (opMode.gamepad1.right_trigger> 0 && revving){
            shooting = true;
        }else {
            shooting = false;
        }

        if (revving && !shooting){
            shooter = true;
            leftClamp.setPosition(0.725);
            rightClamp.setPosition(0.275);
            intakeStrength = 0.3;
            transferPower = -1;
        }else if (shooting) {
            shooter = true;
            leftClamp.setPosition(0.725);
            rightClamp.setPosition(0.275);
            intakeStrength = 1;
            transferPower = 1;
        }else {
            leftClamp.setPosition(0.45);
            rightClamp.setPosition(0.55);
            intakeStrength = 1;
            transferPower = 0;
            shooter = false;
        }

        shooterPower = shooter ? 1 : 0;
        intakePower = intakeStrength * intakeDir;
        transferPower = transferPower;

        shooterDriveLeft.setPower(shooterPower);
        shooterDriveRight.setPower(shooterPower);
        intakeDrive.setPower(intakePower);
        transferDrive.setPower(transferPower);
    }
}