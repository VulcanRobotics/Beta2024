package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.TalonUtil;
import org.littletonrobotics.junction.Logger;

/** Subsystem class for the shooter and intake. */
public class ShooterSubsystem extends SubsystemBase {
  public TalonFX leftMotor = new TalonFX(ShooterConstants.kGuideMotorPort, "rio");
  public TalonFX rightMotor = new TalonFX(ShooterConstants.kFollowMotorPort, "rio");
  public Spark lights = new Spark(9);
  private SysIdRoutine sysId;

  private VelocityVoltage m_request = new VelocityVoltage(0);
  private Follower m_follow = new Follower(ShooterConstants.kGuideMotorPort, false);

  CANSparkMax intakeUpperMotor =
      new CANSparkMax(ShooterConstants.kIntakeUpperMotor, MotorType.kBrushless);

  CANSparkMax feederMotor = new CANSparkMax(ShooterConstants.kFeederMotor, MotorType.kBrushless);

  public DigitalInput intakeSensor = new DigitalInput(ShooterConstants.kPhotogatePort);

  public double savedShootSpeed = ShooterConstants.kShooterTargetVelocity; // 1.0

  public float savedIntakeUpperSpeed = 0;
  public float savedIntakeLowerSpeed = 0;

  public boolean upToSpeed;

  public boolean intakeOverride;

  public static boolean hasNote;

  public ShooterSubsystem() {
    var slot0Configs = new Slot0Configs();
    // These values need to be tuned and put into constants
    slot0Configs.kV = 0.12;
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    leftMotor.getConfigurator().apply(slot0Configs, 0.050);
    rightMotor.getConfigurator().apply(slot0Configs, 0.050);
    TalonUtil.setCoastMode(leftMotor);
    TalonUtil.setCoastMode(rightMotor);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  public void runVolts(double volts) {
    leftMotor.setControl(new VoltageOut(volts));
    rightMotor.setControl(new VoltageOut(volts));
  }

  public void SetFeeder(float speed) {
    // speed = (Constants.name == "Swift") ? -speed : -speed;
    feederMotor.set(-speed);
  }

  public void SetIntake(float speed) {
    intakeUpperMotor.set(speed);
  }

  public void SetIntakeMotor(String motor, float speed) {
    intakeUpperMotor.set(speed);
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

  public void setLED(double color) {
    lights.set(color);
  }

  public boolean hasNote() {
    return intakeSensor.get();
  }

  @Override
  public void periodic() {

    hasNote = intakeSensor.get();

    if (intakeSensor.get()) {
      setLED(-0.07);
    } else if (intakeUpperMotor.get() != 0) {
      setLED(-0.25);
    } else {
      setLED(0.87);
    }

    SmartDashboard.putNumber("Avg Shoot Velocity", getAverageShootSpeed());
    // SmartDashboard.putNumber("m1 velocity", leftMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("Shooter up to Speed", upToSpeed);
    SmartDashboard.putBoolean("Has Note", intakeSensor.get());
    SmartDashboard.putBoolean("Intake Override", intakeOverride);

    // SmartDashboard.putNumber("Upper Intake Speed", savedIntakeUpperSpeed);
    // SmartDashboard.putNumber("Lower Intake Speed", savedIntakeLowerSpeed);

    Logger.recordOutput("Shooter/Avg Shooter Velocity", getAverageShootSpeed());
    Logger.recordOutput("Shooter/Shooter up to speed", upToSpeed);
    Logger.recordOutput("Shooter/Photogate", intakeSensor.get());
    Logger.recordOutput("Shooter/Feeder Motor Speed", feederMotor.get());
  }
}
