package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="thing")
public class Main extends OpMode {
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backRight;
    private DcMotorEx backLeft;
    final private double speed = 10;

    @Override
    public void init() {
        omniWheelsSetup();
    }

    @Override
    public void loop() {
        omniWheelsLoop();
        telemetry.update();
    }

    void omniWheelsSetup() {
        frontRight = hardwareMap.get(DcMotorEx.class, "m2");
        frontLeft = hardwareMap.get(DcMotorEx.class, "m3");
        backRight = hardwareMap.get(DcMotorEx.class, "m1");
        backLeft = hardwareMap.get(DcMotorEx.class, "m0");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients omniPID = new PIDFCoefficients(
                1,
                1,
                1,
                0
        );
        frontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, omniPID);
        backLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, omniPID);
        frontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, omniPID);
        backRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, omniPID);
    }

    void omniWheelsLoop() { // TODO: remember/relearn how this works
        double rightPower = speed * (gamepad1.left_stick_y - gamepad1.right_stick_x); // rotation
        double leftPower = speed * (gamepad1.left_stick_y + gamepad1.right_stick_x); // and fw/bw
        double power = speed * gamepad1.left_stick_x; // strafe
        frontRight.setPower(Range.clip(rightPower + power, -1, 1));
        backRight.setPower(Range.clip(rightPower - power, -1, 1));
        frontLeft.setPower(Range.clip(leftPower - power, -1, 1));
        backLeft.setPower(Range.clip(leftPower + power, -1, 1));

        telemetry.addData("right_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_x", gamepad1.right_stick_x);
        telemetry.addData("frontLeft wheel", frontLeft.getVelocity());
        telemetry.addData("frontRight wheel", frontRight.getVelocity());
        telemetry.addData("backLeft wheel", backLeft.getVelocity());
        telemetry.addData("backRight wheel", backRight.getVelocity());
    }
}

class PID {
    private final double kP;
    private final double kI;
    private final double kD;
    private double integral = 0;
    private double previousError = 0;
    private final ElapsedTime loopTime;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        loopTime = new ElapsedTime();
        loopTime.reset();
    }

    public double calculatePID(double target, double current) {
        double deltaTime = loopTime.seconds();
        loopTime.reset();

        double error = target - current;
        integral += error * deltaTime;
        double derivative = deltaTime > 0 ? (error - previousError) / deltaTime : 0;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }
}