package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "motortest (Blocks to Java)")
public class MotorTest extends LinearOpMode {
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            TimeUnit.SECONDS.wait(5);
            Print("Starting");
            SetPowerAll(0.4);


            TimeUnit.SECONDS.wait(5);
            Print("Stopping");
            SetPowerAll(0);
        }
    }


    public static void Print(String message){
        telemetry.addData("Log", message);
    };
    public static void SetPowerAll(double power){
        for (int i=0;i<4;i++){
            DcMotor motor = hardwareMap.get(DcMotor.class,"m"+i);
            motor.setPower(power);
        }
    }
}