package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "motortest (Blocks to Java)")
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();
        SetPower(1, 0.5);
        SetPower(2, 1);
        SetPower(3, 0.1);
        sleep(2000);
        SetPower(1, 1);
        SetDirection(3, false);
        while (opModeIsActive()) {

        }
    }
    public void Print(String message){
        telemetry.addData("Log", message);
    };
    public void SetPowerAll(double power){
        for (int i=0;i<4;i++){
            DcMotor motor = hardwareMap.get(DcMotor.class,"m"+i);
            motor.setPower(power);
        }
    }
    public void SetPower(int motorId, double power){
        DcMotor motor = hardwareMap.get(DcMotor.class,"m"+motorId);
        motor.setPower(power);
    }
    public void SetDirection(int motorId, boolean forward){
        DcMotor motor = hardwareMap.get(DcMotor.class,"m"+motorId);
        motor.SetDirection(forward ? DcMotor.Direction.FORWARD:DcMotor.Direction.REVERSE);
    }
}