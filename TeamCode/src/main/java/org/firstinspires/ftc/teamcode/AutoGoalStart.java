/*
Copyright 2026 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous(name = "Auto goal start", preselectTeleOp = "Tank controller with aim assist")
public class AutoGoalStart extends LinearOpMode {
    private ShooterController shooterController = new ShooterController(this);
    private ChassisController chassisController = new ChassisController(this);
    private boolean lastDpad = false;

    private boolean redTeam = true;
    private boolean noFire = false;
    private boolean ignoreFinalSet = false;
    private double timeModifier = 1;
    private double powerModifier = 1;

    private int selectedData = 0;
    private int teamModifier = 1;
    @Override
    public void runOpMode() {
        while(opModeInInit()){
            boolean dpad = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
            if (dpad && !lastDpad) {
                if (gamepad1.dpad_down){
                    selectedData += 1;
                }
                if (gamepad1.dpad_up){
                    selectedData -= 1;
                }
                selectedData = (selectedData + 400) % 4;
                if (gamepad1.dpad_right || gamepad1.dpad_left){
                    if (selectedData == 0) redTeam = !redTeam;
                    if (selectedData == 1) ignoreFinalSet = !ignoreFinalSet;
                    if (selectedData == 2) noFire = !noFire;
                    if (selectedData == 3) {
                        if (gamepad1.dpad_right){
                            powerModifier += 0.05;
                        }else {
                            powerModifier -= 0.05;
                        }
                    }
                }
            }
            lastDpad = dpad;
            telemetry.addData((selectedData == 0?">":"") + "Team", redTeam ? "red" : "blue");
            telemetry.addData((selectedData == 1?">":"") + "Ignore final set", ignoreFinalSet);
            telemetry.addData((selectedData == 2?">":"") + "Disable firing", noFire);
            telemetry.addData((selectedData == 3?">":"") + "Distance modifier", powerModifier);
            telemetry.addData("Controls", "Dpad < > to change values, v ^ to select.");
            telemetry.update();
        }
        teamModifier = redTeam?1:-1;
        shooterController.init();
        chassisController.init();
        waitForStart();
        
        shooterController.setWeakIdle();
        
        runWithModifier(0,-0.4,0);
        sleepThenPause(900);
        
        turn45(teamModifier);
        
        runWithModifier(0,-0.6,0);
        sleepThenPause(1200);
        
        //fire
        fire(400);
        //
        turn50(teamModifier);
        
        runWithModifier(0,0.4,0);
        sleepThenPause(1900);
        shooterController.setWeakIdle();
        //
        runWithModifier(0,-0.4,0);
        sleepThenPause(1900);
        
        turn50(-teamModifier);
        
        fire(400);
        if (ignoreFinalSet) return;
        //
        turn50(teamModifier);
        
        runWithModifier(Math.toRadians(90),0.6 * teamModifier,0);
        sleepThenPause(900);
        //
        runWithModifier(0,0.4,0);
        sleepThenPause(2000);
        shooterController.setWeakIdle();
        //
        runWithModifier(0,-0.3,0);
        sleepThenPause(300);
        runWithModifier(Math.toRadians(-90),0.6 * teamModifier,0);
        sleepThenPause(300);
        runWithModifier(0,0.3,0);
        sleepThenPause(500);
        //
        runWithModifier(0,-0.4,0);
        sleepThenPause(2000);
        //
        
        
        turn50(-teamModifier);
        
        runWithModifier(0,0.4,0);
        sleepThenPause(600);
        
        runWithModifier(Math.toRadians(-90),0.6 * teamModifier,0);
        sleepThenPause(300);
        
        fire(420);
        shooterController.setWeakIdle();
    }
    void turn45(int direction){
        runWithModifier(0,0,0.3*direction);
        sleepThenPause(700);
    }
    void turn50(int direction){
        runWithModifier(0,0,0.3*direction);
        sleepThenPause(730);
    }
    void pause(){
        runWithModifier(0,0,0);
        sleep(150);
    }
    void sleepThenPause(int time){
        sleepWithModifier(time);
        pause();
    }
    void fire(double vel){
        if (noFire){
            shooterController.setUnclogIdle();
        } else {
            shooterController.setRevving(vel);
        }
        sleep(1500);
        if (!noFire){
            shooterController.setShooting(vel);
        }
        sleep(1500);
        shooterController.setIdle();
        sleepThenPause(200);
    }
    void sleepWithModifier(int time){
        sleep(((Double)(time * timeModifier)).intValue());
    }
    void runWithModifier(double moveAngle, double moveMagnitude, double turnPower) {
        chassisController.run(moveAngle,moveMagnitude * powerModifier, turnPower * powerModifier);
    }
}
