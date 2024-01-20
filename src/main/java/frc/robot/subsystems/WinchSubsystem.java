package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchSubsystem extends SubsystemBase {
  CANSparkMax winchMotor = new CANSparkMax(15, MotorType.kBrushless);

  public void setWinchSpeed(double speed) {
    winchMotor.set(speed);
  }
}
