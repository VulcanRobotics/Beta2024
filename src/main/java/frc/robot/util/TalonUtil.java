package frc.robot.util;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonUtil {

  /*
   * @param
   * This enables hard brake mode on motor.
   * Use setCoastMode to enable neutral.
   */
  public static void setBrakeMode(TalonFX motor) {
    var config = new MotorOutputConfigs();
    config.NeutralMode = NeutralModeValue.Brake;
    motor.getConfigurator().apply(config);
  }

  /*
   * @param
   * This enables coast mode on motor.
   * Use setBrakeMode to enable hard brake mode.
   */
  public static void setCoastMode(TalonFX motor) {
    var config = new MotorOutputConfigs();
    config.NeutralMode = NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }
}
