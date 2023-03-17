package frc.robot.util;

public class Limiter {
  public static double deadzone(double input, double zone) {
    double absInput = Math.abs(input);
    if (absInput <= zone) {
      return 0;
    } else {
      return input;
    }
  }

  public static double bound(double unbounded, double lowerLimit, double upperLimit) {
    return Math.min(Math.max(unbounded, lowerLimit), upperLimit);
  }

  public static double bound(double input, double limit) {
    return bound(input, -limit, limit);
  }
}
