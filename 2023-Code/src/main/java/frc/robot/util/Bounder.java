package frc.robot.util;

public class Bounder {
    public static double deadzone(double input, double zone) {
        double absInput = Math.abs(input);
        if (absInput <= zone) {
            return 0;
        } else {
            return input;
        }
    }
}
