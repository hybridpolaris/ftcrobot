package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;



@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")

public class TeleopTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status","Init");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotor.class, "m0");
        rightDrive = hardwareMap.get(DcMotor.class, "m1");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        
        double turn = 0;
        double drive = 0;
        boolean shoot = false;
        while (opModeIsActive()) {
            if (turn != gamepad1.right_stick_x){
                telemetry.addData("Turn", gamepad1.right_stick_x);
            }
            if (drive != -gamepad1.left_stick_y){
                telemetry.addData("Drive", -gamepad1.left_stick_y);
            }
            if (shoot != gamepad1.a){
                telemetry.addData("Shoot", gamepad1.a);
            }
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;
            shoot = gamepad1.a;

            double leftPower;
            double rightPower;

            leftPower    = Range.clip(turn + drive * Math.abs(1-turn), -1.0, 1.0) ;
            rightPower   = Range.clip(turn + drive * Math.abs(1-turn), -1.0, 1.0) ;
            

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            telemetry.update();
        }
    }
}
