package frc.robot.util;

public enum LEDColor {
  ORANGE(255, 15, 0),
  PURPLE(210, 25, 210),
  YELLOW(255, 100, 0),
  RED(255, 0, 0),
  BLUE(0, 0, 255),
  INDETERMINATE(-1, -1, -1);

  private final int m_r;
  private final int m_g;
  private final int m_b;

  private LEDColor(int p_r, int p_g, int p_b) {
    m_r = p_r;
    m_g = p_g;
    m_b = p_b;
  }

  public int getR() {
    return m_r;
  }

  public int getG() {
    return m_g;
  }

  public int getB() {
    return m_b;
  }
}
