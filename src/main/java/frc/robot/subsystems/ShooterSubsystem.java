package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax leftMotor = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkMax rightMotor = new CANSparkMax(16, MotorType.kBrushless);

  CANSparkMax intakeMotor = new CANSparkMax(18, MotorType.kBrushless);
  CANSparkMax feederMotor = new CANSparkMax(19, MotorType.kBrushless);

  public DigitalInput intakeSensor = new DigitalInput(0);

  public float savedShootSpeed = (float) 1.0;

  public void SetFeeder(float speed) {
    feederMotor.set(-speed);
  }

  public void SetIntake(float speed) {
    intakeMotor.set(speed);
  }

  public void SetShooter(float speed) {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
}
