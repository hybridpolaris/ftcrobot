package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.BatteryChecker;
import java.lang.Math;
import java.util.Vector;



@TeleOp(name="Joystick controls")
//Alternative TeleOp mode

public class Joystick extends LinearOpMode {

    private ShooterController shooterController = new ShooterController(this);
    private ChassisController chassisController = new ChassisController(this);
    @Override
    public void runOpMode() {
        waitForStart();
        
        shooterController.init();
        chassisController.init();

        telemetry.addData("Status","Init joystick controller");
        telemetry.update();

        while (opModeIsActive()) {
            double x_control = gamepad1.left_stick_x + ((gamepad1.dpad_right?1:0)-(gamepad1.dpad_left?1:0));
            double y_control = -gamepad1.left_stick_y + ((gamepad1.dpad_up?1:0)-(gamepad1.dpad_down?1:0));
            double turn  =  gamepad1.right_stick_x;

            double driveAngle = Math.atan2(x_control, y_control);
            double driveMagnitude = Math.sqrt(Math.pow(x_control,2) + Math.pow(y_control,2));

            shooterController.run();
            chassisController.run(driveAngle, driveMagnitude, turn);

            telemetry.addData("Drive angle", Math.toDegrees(driveAngle));
            telemetry.addData("Drive power", driveMagnitude);
            telemetry.addData("Turn", turn);
            telemetry.addData("backright", chassisController.backRightPower);
            telemetry.addData("backleft", chassisController.backLeftPower);
            telemetry.addData("frontright", chassisController.frontRightPower);
            telemetry.addData("frontleft", chassisController.frontLeftPower);
            telemetry.addData("revving", shooterController.revving);
            telemetry.addData("shooting", shooterController.shooting);
            telemetry.update();

            if (gamepad1.x) {
                break; //Killswitch
            }
        }
    }
}
