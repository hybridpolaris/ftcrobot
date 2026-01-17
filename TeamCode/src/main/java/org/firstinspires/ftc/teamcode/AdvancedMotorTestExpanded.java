package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Advanced Motor Test Expanded")

public class AdvancedMotorTestExpanded extends LinearOpMode {
  @Override
  public void runOpMode() {
    telemetry.addData("Init", "Version 1.0");
    telemetry.update();
    waitForStart();

    double[] directions = new double[8];
    int selectedMotor = 0;
    double step = 1;
    boolean previousInputDown = false;

    Arrays.fill(directions, 0);
    while (opModeIsActive()) {
      boolean inputDown = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
      if (inputDown && !previousInputDown) {
        if (gamepad1.dpad_down) {
          selectedMotor += 1;
        } else if (gamepad1.dpad_up) {
          selectedMotor -= 1;
        }
        selectedMotor = (selectedMotor + (directions.length + 1)*100) % (directions.length + 1);

        if (gamepad1.dpad_right) {
          if (selectedMotor == directions.length) {
            step += 0.1;
          } else {
            directions[selectedMotor] += step;
          }
        } else if (gamepad1.dpad_left) {
          if (selectedMotor == directions.length) {
            step -= 0.1;
          } else {
            directions[selectedMotor] -= step;
          }
        }
        if (selectedMotor == directions.length) {
          step = Range.clip(step, 0.1, 1);
        } else {
          directions[selectedMotor] = Range.clip(directions[selectedMotor], -1, 1);
        }
      }
      for (int i = 0; i < directions.length; i++) {
        if (gamepad1.a) {
          directions[i] = 1;
        }
        if (gamepad1.b) {
          directions[i] = -1;
        }
        if (gamepad1.x) {
          directions[i] = 0;
        }

        if (i < 4) {
          DcMotor motor = hardwareMap.get(DcMotor.class, "m" + i);
          motor.setPower(Math.abs(directions[i]));
          if (directions[i] > 0) {
            motor.setDirection(DcMotor.Direction.FORWARD);
          } else {
            motor.setDirection(DcMotor.Direction.REVERSE);
          }
          telemetry.addData(((selectedMotor == i) ? "> Motor " : "Motor ").concat(String.valueOf(i)), directions[i]);
        } else {
          int j = i-4;
          Servo servo = hardwareMap.get(Servo.class, "s" + j);
          servo.scaleRange(0,1);
          servo.setPosition(directions[i]/2 + 0.5);
          telemetry.addData(((selectedMotor == i) ? "> Servo " : "Servo ").concat(String.valueOf(j)), directions[i]);
        }
      }
      telemetry.addData((selectedMotor == directions.length) ? "> Step " : "Step ", step);
      telemetry.update();

      previousInputDown = inputDown;
    }

  }
}