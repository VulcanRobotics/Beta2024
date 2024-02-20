package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public TalonFX leftMotor = new TalonFX(17, "rio");
  public TalonFX rightMotor = new TalonFX(16, "rio");

  private VelocityVoltage m_request = new VelocityVoltage(0);
  private Follower m_follow = new Follower(17, false);

  CANSparkMax intakeMotor = new CANSparkMax(18, MotorType.kBrushless);
  CANSparkMax feederMotor = new CANSparkMax(19, MotorType.kBrushless);

  public DigitalInput intakeSensor = new DigitalInput(0);
  public boolean toggleShooter = false;

  public boolean upToSpeed;
  public double savedShootSpeed = 1.0;

  public ShooterSubsystem() {}

  public void SetFeeder(float speed) {
    speed = (Constants.name == "Swift") ? speed : -speed;
    feederMotor.set(speed);
  }

  public void SetIntake(float speed) {
    intakeMotor.set(speed);
  }

  public void SetShooter(double speed) {
    leftMotor.set(-speed);
    rightMotor.setControl(m_follow);
  }

  public void setShooterVelocity(double velocity) {
    m_request = m_request.withVelocity(velocity);
    leftMotor.setControl(m_request);
    rightMotor.setControl(m_follow);
  }

  public double getAverageShootSpeed() {
    double avgSpeed =
        (leftMotor.getVelocity().getValueAsDouble() + rightMotor.getVelocity().getValueAsDouble())
            / 2;
    return -avgSpeed;
  }

  public void periodic() {

    if (toggleShooter == true) {
      leftMotor.set(-1);
      rightMotor.set(-1);
    }
    SmartDashboard.putBoolean("Shooter Toggle", toggleShooter);
    SmartDashboard.putNumber("Avg Shoot Velocity", getAverageShootSpeed());
    SmartDashboard.putBoolean("Shooter up to speed", upToSpeed);
  }
}
