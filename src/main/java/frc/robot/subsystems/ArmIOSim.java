// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private static final double autoStartAngle = Units.degreesToRadians(80.0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(2),
          (9.0 / 1.0) * (6.0 / 1.0) * (80.0 / 15.0),
          1.06328,
          1.0,
          Math.toRadians(0),
          Math.toRadians(90),
          false,
          Units.degreesToRadians(0.0));

  private final PIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  public ArmIOSim() {
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    // Reset at start of auto
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    sim.update(0.02);

    inputs.armMotor1Pos = sim.getAngleRads();
    inputs.armMotor2Pos = sim.getAngleRads();
    inputs.armCancoderPos = sim.getAngleRads();

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void setArmMotor1Request(MotionMagicVoltage request) {
    sim.setInputVoltage(request.FeedForward);
  }

  @Override
  public void setArmMotor2Follow(Follower follow) {}

  @Override
  public void setArmMotor1BrakeMode(boolean enable) {}

  @Override
  public void setArmMotor2BrakeMode(boolean enable) {}

  @Override
  public void setArmMotor1Pos(double value) {}

  @Override
  public void setArmMotor2Pos(double value) {}

  @Override
  public void setArmMotor1Speed(double speed) {
    sim.setInput(speed);
  }

  @Override
  public void setArmMotor2Speed(double speed) {}
}
