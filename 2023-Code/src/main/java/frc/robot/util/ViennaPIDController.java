package frc.robot.util;

public class ViennaPIDController {

  private double m_P;
  private double m_I;
  private double m_D;

  private double error;
  private double integral;
  private double derivative;
  private double previousError = 0;
  private double dt = 20;

  public ViennaPIDController() {}

  public ViennaPIDController(double p_P) {
    m_P = p_P;
    m_I = 0;
    m_D = 0;
  }

  public ViennaPIDController(double p_P, double p_I) {
    m_P = p_P;
    m_I = p_I;
  }

  public ViennaPIDController(double p_P, double p_I, double p_D) {
    m_P = p_P;
    m_I = p_I;
    m_D = p_D;
  }

  public double getP() {
    return m_P;
  }

  public double getI() {
    return m_I;
  }

  public double getD() {
    return m_D;
  }

  public void setP(double p_P) {
    m_P = p_P;
  }

  public void setI(double p_I) {
    m_I = p_I;
  }

  public void setD(double p_D) {
    m_D = p_D;
  }

  public double run(double realPos, double desiredPos) {
    error = desiredPos - realPos;

    /*Integral calculation */
    integral += error * dt;

    /*Derivative calculation */
    derivative = (error - previousError) / dt;
    previousError = error;

    return m_P * error + m_I * integral + m_D * derivative;
  }
}
