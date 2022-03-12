package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class JoystickSubsystem extends SubsystemBase {
  private static class JoystickConfig {
    public int lx;
    public int ly;

    public int rx;
    public int ry;

    public int y;
    public int x;
    public int a;
    public int b;

    public int lb1;
    public int rb1;

    public int lb2;
    public int rb2;

    public JoystickConfig(int lx, int ly, int rx, int ry, int y, int x, int a, int b, int lb1,
        int rb1, int lb2, int rb2) {
      this.lx = lx;
      this.ly = ly;
      this.rx = rx;
      this.ry = ry;
      this.y = y;
      this.x = x;
      this.a = a;
      this.b = b;
      this.lb1 = lb1;
      this.rb1 = rb1;
      this.lb2 = lb2;
      this.rb2 = rb2;
    }
  }

  private final Joystick joystick;
  private final JoystickConfig config;

  public static JoystickConfig logitechConfig =
      new JoystickConfig(1, 0, 5, 4, 4, 3, 1, 2, 5, 6, 2, 3);

  public JoystickSubsystem(Joystick joystick, JoystickConfig config) {
    this.joystick = joystick;
    this.config = config;
  }

  public Vector2d getLeftStick() {
    return getStick(config.lx, config.ly);
  }

  public Vector2d getRightStick() {
    return getStick(config.rx, config.ry);
  }

  public JoystickButton getA() {
    return new JoystickButton(joystick, config.a);
  }

  public JoystickButton getB() {
    return new JoystickButton(joystick, config.b);
  }

  public JoystickButton getX() {
    return new JoystickButton(joystick, config.x);
  }

  public boolean isXPressed() {
    return joystick.getRawButton(config.x);
  }

  public JoystickButton getY() {
    return new JoystickButton(joystick, config.y);
  }

  public JoystickButton getLB1() {
    return new JoystickButton(joystick, config.lb1);
  }


  public double getLB2() {
    return joystick.getRawAxis(config.lb2);
  }

  public DoubleSupplier getLB2Supplier() {
    return () -> joystick.getRawAxis(config.lb2);
  }

  public JoystickButton getRB1() {
    return new JoystickButton(joystick, config.rb1);
  }

  public DoubleSupplier getRB2Supplier() {
    return () -> joystick.getRawAxis(config.rb2);
  }

  public double getRB2() {
    return joystick.getRawAxis(config.rb2);
  }

  private Vector2d getStick(int axisVertical, int axisHorizontal) {
    double x = joystick.getRawAxis(axisHorizontal);
    double y = -joystick.getRawAxis(axisVertical);

    if(Math.abs(x) < 0.05) x = 0;
    if(Math.abs(y) < 0.05) y = 0;

    return new Vector2d(x, y);
  }

  public int getDpad() {
    return joystick.getPOV(0);
  }

  public boolean isDpadLeft() {
    return getDpad() == 90;
  }
  
  public boolean isDpadRight() {
    return getDpad() == 270;
  }
}
