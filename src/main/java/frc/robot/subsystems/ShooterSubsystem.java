package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX leftMotor = new TalonFX(17, "rio");
  TalonFX rightMotor = new TalonFX(16, "rio");

  CANSparkMax intakeMotor = new CANSparkMax(18, MotorType.kBrushless);
  CANSparkMax feederMotor = new CANSparkMax(19, MotorType.kBrushless);

  public DigitalInput intakeSensor = new DigitalInput(0);

  public float savedShootSpeed = (float) 1.0; // 1.0

  public void SetFeeder(float speed) {
    feederMotor.set(-speed);
  }

  public void SetIntake(float speed) {
    intakeMotor.set(speed);
  }

  public void SetShooter(float speed) {
    leftMotor.set(-speed);
    rightMotor.set(-speed);
  }

  public double getAverageShootSpeed() {
    double avgSpeed =
        (leftMotor.getVelocity().getValueAsDouble() + rightMotor.getVelocity().getValueAsDouble())
            / 2;
    return -avgSpeed;
  }

  public void periodic() {
    SmartDashboard.putNumber("Avg Shoot Velocity", getAverageShootSpeed());
    SmartDashboard.putNumber("Shooter Speed", savedShootSpeed);
  }
}
