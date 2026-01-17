package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="AdvancedMotorTest")

public class AdvancedMotorTest extends LinearOpMode {
  @Override
  public void runOpMode() {
    telemetry.addData("Init","Version 1.0");
    telemetry.update();
    waitForStart();

    int[] directions = new int[4];
    int selectedMotor = 0;
    double step = 1;
    boolean previousInputDown = false;
    
    Arrays.fill(directions,0);
    while (opModeIsActive()) {
      boolean inputDown = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
      if (inputDown && !previousInputDown){
        if (gamepad1.dpad_down){
          selectedMotor += 1;
        }else if(gamepad1.dpad_up){
          selectedMotor -= 1;
        }
        selectedMotor = (selectedMotor + 500) % 5;

        if (gamepad1.dpad_right){
          if (selectedMotor == 4){
            step += 0.1;
          }else{
            directions[selectedMotor] += step;
          }
        }else if (gamepad1.dpad_left){
          if (selectedMotor == 4){
            step -= 0.1;
          }else{
            directions[selectedMotor] -= step;
          }
        }
        if (selectedMotor == 4){
          step = Range.clip(step ,0.1,1);
        }else{
          directions[selectedMotor] = Range.clip(directions[selectedMotor] ,-1,1);
        }
      }
      for (int i=0;i<4;i++){
        DcMotor motor = hardwareMap.get(DcMotor.class,"m"+i);

        if (gamepad1.a){
          directions[i] = 1;
        }
        if (gamepad1.b){
          directions[i] = -1;
        }
        if (gamepad1.x){
          directions[i] = 0;
        }

        motor.setPower(Math.abs(directions[i]));
        if (directions[i] > 0){
          motor.setDirection(DcMotor.Direction.FORWARD);
        }else{
          motor.setDirection(DcMotor.Direction.REVERSE);
        }
        telemetry.addData(((selectedMotor == i) ? "> Motor " : "Motor " ).concat(String.valueOf(i)), directions[i]);
      }
      telemetry.addData((selectedMotor == 4) ? "> Step " : "Step ", step);
      telemetry.update();

      previousInputDown = inputDown;
    }
    
  }
}