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

  public static double limitAmount(double input, double upperLimit, double lowerLimit) {
    if (input < lowerLimit) {
      input = lowerLimit;
    } else if (input > upperLimit) {
      input = upperLimit;
    }
    return input;
  }

  public static double limitAmount(double input, double limit) {
    return limitAmount(input, -limit, limit);
  }
}
