package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  public TalonFX leftMotor = new TalonFX(ShooterConstants.kGuideMotorPort, "rio");
  public TalonFX rightMotor = new TalonFX(ShooterConstants.kFollowMotorPort, "rio");

  private VelocityVoltage m_request = new VelocityVoltage(0);
  private Follower m_follow = new Follower(ShooterConstants.kGuideMotorPort, false);

  CANSparkMax intakeMotor = new CANSparkMax(ShooterConstants.kIntakeMotor, MotorType.kBrushless);
  CANSparkMax feederMotor = new CANSparkMax(ShooterConstants.kFeederMotor, MotorType.kBrushless);

  public DigitalInput intakeSensor = new DigitalInput(ShooterConstants.kPhotogatePort);
  public boolean toggleShooter = false;
  public double savedShootSpeed = ShooterConstants.kShooterTargetVelocity; // 1.0
  public boolean upToSpeed;

  public ShooterSubsystem() {
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.01;
    slot0Configs.kI = 0.00;
    slot0Configs.kD = 0.00;
    leftMotor.getConfigurator().apply(slot0Configs, 0.050);
    rightMotor.getConfigurator().apply(slot0Configs, 0.050);
  }

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
    m_request.Slot = 0;
    m_request = m_request.withVelocity(-velocity);
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
