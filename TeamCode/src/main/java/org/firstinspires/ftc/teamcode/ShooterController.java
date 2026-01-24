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
    private boolean last_a = false;
    private boolean last_b = false;
    private boolean last_y = false;
    private double intakeStrength = 1;

    public double shooterStrength = 1;
    public double shooterDps = 300;

    public boolean usePower = false;
    public double shooterPower;
    public double shooterOutputDps;
    public double intakePower;
    public double transferPower;

    public double shooterLeftPower;
    public double shooterRightPower;
    public double shooterLeftVelocity;
    public double shooterRightVelocity;
    public static final PIDFCoefficients coefficient = new PIDFCoefficients(30, 0, 0, 0);

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

        ((DcMotorEx) shooterDriveLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);
        ((DcMotorEx) shooterDriveRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficient);

        leftClamp.setDirection(Servo.Direction.FORWARD);
        rightClamp.setDirection(Servo.Direction.FORWARD);
        leftClamp.scaleRange(0, 1);
        rightClamp.scaleRange(0, 1);

        opMode.telemetry.addData("Status", "Init ShooterController module");
    }
    public void run() {
        if (opMode.gamepad1.a && !last_a) {
            if (usePower) {
                shooterStrength += 0.05;
            } else {
                shooterDps += 25;
            }
        }
        if (opMode.gamepad1.b && !last_b) {
            if (usePower) {
                shooterStrength -= 0.05;
            } else {
                shooterDps -= 25;
            }
        }
        if (opMode.gamepad1.y && !last_y) {
            usePower = !usePower;
        }
        shooterStrength = Range.clip(shooterStrength, 0, 1);
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
            shooter = true;
            leftClamp.setPosition(0.725);
            rightClamp.setPosition(0.275);
            intakeStrength = 0.3;
            transferPower = -1;
        } else if (shooting) {
            shooter = true;
            leftClamp.setPosition(0.725);
            rightClamp.setPosition(0.275);
            intakeStrength = 1;
            transferPower = 1;
        } else {
            leftClamp.setPosition(0.45);
            rightClamp.setPosition(0.55);
            intakeStrength = 1;
            transferPower = 0;
            shooter = false;
        }

        intakePower = intakeStrength * intakeDir;
        transferPower = transferPower;

        if (usePower) {
            shooterPower = shooter ? shooterStrength : 0;
            shooterDriveLeft.setPower(shooterPower);
            shooterDriveRight.setPower(shooterPower * 0.955);
        } else {
            shooterOutputDps = shooter ? shooterDps : 0;
            ((DcMotorEx)shooterDriveLeft).setVelocity(shooterOutputDps,AngleUnit.DEGREES);
            ((DcMotorEx)shooterDriveRight).setVelocity(shooterOutputDps,AngleUnit.DEGREES);
        }

        intakeDrive.setPower(intakePower);
        transferDrive.setPower(transferPower);

        shooterLeftVelocity = ((DcMotorEx) shooterDriveLeft).getVelocity(AngleUnit.DEGREES);
        shooterRightVelocity = ((DcMotorEx) shooterDriveRight).getVelocity(AngleUnit.DEGREES);
        shooterLeftPower = shooterDriveLeft.getPower();
        shooterRightPower = shooterDriveRight.getPower();
    }
}