package org.firstinspires.ftc.teamcode;

public class Util {
    public static String toPercentage(double d){
        return String.valueOf(Math.round(((Double)(shooterController.shooterStrength * 100)).floatValue()))+"%";
    }
}
