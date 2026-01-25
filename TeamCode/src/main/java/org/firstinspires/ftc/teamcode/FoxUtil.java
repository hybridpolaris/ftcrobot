package org.firstinspires.ftc.teamcode;

public class FoxUtil {
    public static String toPercentage(double d){
        return String.valueOf(Math.round(((Double)(shooterController.shooterStrength * 100)).floatValue()))+"%";
    }
    public static double angleDiff(double a, double b){
        double d = b - a;
        return Math.abs(d) < 180 ? d:(d > 0? d-360:d+360);
    }
}
