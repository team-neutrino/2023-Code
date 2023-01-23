// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int JOYSTICK_RIGHT = 0;
    public static final int JOYSTICK_LEFT = 1;
    public static final int XBOX = 4;
  }

  public static class PDPConstants {
    public static final int PDP_CAN_ID = 0;
  }

  public static class MotorConstants {
    public static final int INTAKEMOTOR1 = 21; // intake motors have '2' in front of actual id
    public static final int INTAKEMOTOR2 = 22;
    public static final int INTAKEMOTOR3 = 23; // not used currently, may be in the future
    public static final int RMOTOR1 = 11;
    public static final int RMOTOR2 = 12;
    public static final int LMOTOR1 = 13;
    public static final int LMOTOR2 = 14;
  }

  public static class PneumaticsConstants {
    public static final int INTAKE_PCM = 0;
    public static final int GRABBER = 1;
    public static final int SOLENOID_FRONT = 2;
    public static final int SOLENOID_BACK = 3;
  }

  public static class VariableConstants {
    public static final double DEADZONE = 0.1;
  }
}
