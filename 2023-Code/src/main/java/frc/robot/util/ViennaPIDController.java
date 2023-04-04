package frc.robot.util;

import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ViennaPIDController {

  private double m_p;
  private double m_i;
  private double m_d;
  private double m_ff;

  private double m_iState;

  private double previousError = 0;

  public ViennaPIDController() {}

  public ViennaPIDController(double p_p) {
    m_p = p_p;
    m_i = 0;
    m_d = 0;
    m_ff = 0;
  }

  public ViennaPIDController(double p_p, double p_i) {
    m_p = p_p;
    m_i = p_i;
    m_d = 0;
    m_ff = 0;
  }

  public ViennaPIDController(double p_p, double p_i, double p_d) {
    m_p = p_p;
    m_i = p_i;
    m_d = p_d;
    m_ff = 0;
  }

  public ViennaPIDController(double p_p, double p_i, double p_d, double p_ff) {
    m_p = p_p;
    m_i = p_i;
    m_d = p_d;
    m_ff = p_ff;
  }

  public double getP() {
    return m_p;
  }

  public double getI() {
    return m_i;
  }

  public double getD() {
    return m_d;
  }

  public double getFF() {
    return m_ff;
  }

  public void setP(double p_p) {
    m_p = p_p;
  }

  public void setI(double p_i) {
    m_i = p_i;
  }

  public void setD(double p_d) {
    m_d = p_d;
  }

  public void setFF(double p_ff) {
    m_ff = p_ff;
  }

  public double run(double realPos, double desiredPos) {
    double error = desiredPos - realPos;

    /*Integral calculation */
    m_iState += error * PIDConstants.dt;

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    double ff = desiredPos * m_ff;

    double output = m_p * error + m_i * m_iState + m_d * derivative + ff;

    return Limiter.bound(output, PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);
  }

  public double run(double realPos, double desiredPos, double zone) {
    double error = desiredPos - realPos;
    error = Limiter.deadzone(error, zone);

    /*Integral calculation */
    m_iState += error * PIDConstants.dt;

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    /* Feedforward calculation */
    double ff = desiredPos * m_ff;

    double output = m_p * error + m_i * m_iState + m_d * derivative + ff;
    return Limiter.bound(output, PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);
  }

  public double armRun(double realPos, double desiredPos, double armExtension) {
    double error = desiredPos - realPos;

    /*Integral calculation */
    m_iState += error * PIDConstants.dt;

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    double pAlteration =
    PIDConstants.ARM_EXTENSION_P * armExtension/ TelescopeConstants.TELESCOPE_EXTEND_LIMIT;

    /* Feedforward calculation */
    double ff = - m_ff * Math.sin(ArmSubsystem.encoderToRadians(realPos)) + pAlteration;

    System.out.println("degrees " + (180/Math.PI)*ArmSubsystem.encoderToRadians(realPos));

    // System.out.println("ff: " + m_ff);

    double output = (pAlteration + m_p) * error + m_i * m_iState + m_d * derivative + ff;
    System.out.println("output: " + output);
    return Limiter.bound(output, PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);
  }
}
