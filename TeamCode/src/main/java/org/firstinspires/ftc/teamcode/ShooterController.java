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



public class ShooterController {
    private LinearOpMode opMode;

    private DcMotor shooterDrive = null;
    private DcMotor intakeDrive = null;
    private DcMotor transferDrive = null;
    private Servo rightClamp = null;
    private Servo leftClamp = null;

    private double intakeDir = 0;
    private boolean shooter = false;
    private boolean transfer = false;

    private boolean last_right_trigger = false;
    private boolean last_right_bumper = false;
    private boolean last_left_pair = false;
    //private long transferTimestamp = System.currentTimeMillis();
    private double intakeStrength = 0.5;

    public double shooterPower;
    public double intakePower;
    public double transferPower;
    public ShooterController(LinearOpMode _opMode) {
        opMode = _opMode;
    }
    public void init() {
        shooterDrive = opMode.hardwareMap.get(DcMotor.class, "m3");
        intakeDrive = opMode.hardwareMap.get(DcMotor.class, "em2");
        transferDrive = opMode.hardwareMap.get(DcMotor.class, "m2");
        leftClamp = opMode.hardwareMap.get(Servo.class, "s0");
        rightClamp = opMode.hardwareMap.get(Servo.class, "es0");

        shooterDrive.setDirection(DcMotor.Direction.REVERSE);
        intakeDrive.setDirection(DcMotor.Direction.FORWARD);
        transferDrive.setDirection(DcMotor.Direction.FORWARD);
        
        leftClamp.setDirection(Servo.Direction.FORWARD);
        rightClamp.setDirection(Servo.Direction.FORWARD);
        leftClamp.scaleRange(0,1);
        rightClamp.scaleRange(0,1);
        
        opMode.telemetry.addData("Status","Init ShooterController module");
    }
    public void run() {
        if ((last_right_trigger != (opMode.gamepad1.right_trigger > 0)) && (opMode.gamepad1.right_trigger > 0)) {
            shooter = !shooter;
        }
        last_right_trigger = opMode.gamepad1.right_trigger > 0;
        if (last_left_pair != (opMode.gamepad1.left_bumper || (opMode.gamepad1.left_trigger > 0))) {
            if (opMode.gamepad1.left_bumper) {
                intakeDir = intakeDir == 1 ? 0 : 1;
            }
            if ((opMode.gamepad1.left_trigger > 0)) {
                intakeDir = intakeDir == -1 ? 0 : 1;
            }
        }
        last_left_pair = opMode.gamepad1.left_bumper || (opMode.gamepad1.left_trigger > 0);

        if (opMode.gamepad1.right_bumper) {
            /*if (!last_right_bumper) {
                transferTimestamp = System.currentTimeMillis() + 1000;
            }
            if (System.currentTimeMillis() > transferTimestamp) {
                transferTimestamp = System.currentTimeMillis();
            }*/
            if (!last_right_bumper) {
                transfer = !transfer;
            }
        }
        last_right_bumper = opMode.gamepad1.right_bumper;
        //transfer = System.currentTimeMillis() <= transferTimestamp;
        if (transfer){
            intakeStrength = 1;
            leftClamp.setPosition(0.725);
            rightClamp.setPosition(0.275);
        }else{
            intakeStrength = 0.5;
            leftClamp.setPosition(0.45);
            rightClamp.setPosition(0.65);
        }

        shooterPower = shooter ? 1 : 0;
        intakePower = intakeStrength * intakeDir;
        transferPower = transfer ? 1 : 0;

        shooterDrive.setPower(shooterPower);
        intakeDrive.setPower(intakePower);
        transferDrive.setPower(transferPower);
    }
}