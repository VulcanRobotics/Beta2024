// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.TalonUtil;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armMotor_1;
  private final TalonFX armMotor_2;
  private final CANcoder cancoder;

  private final StatusSignal<Double> armMotor1Pos;
  private final StatusSignal<Double> armMotor2Pos;
  private final StatusSignal<Double> armCancoderPos;

  // Gear ratios for SDS MK4i L2, adjust as necessary
  private final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private Follower m_follow = new Follower(ArmConstants.kGuideMotorPort, true);

  public ArmIOTalonFX() {
    armMotor_1 = new TalonFX(15, "rio");
    armMotor_2 = new TalonFX(14, "rio");
    cancoder = new CANcoder(12, "rio");

    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    // Set up PID and Feedforward config for the motors.
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.6; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 0.05; // A velocity error of 1 rps results in 0.1 V output

    armMotor_1.getConfigurator().apply(talonFXConfigs);
    armMotor_2.getConfigurator().apply(talonFXConfigs);
    TalonUtil.setBrakeMode(armMotor_1);
    TalonUtil.setBrakeMode(armMotor_2);
    armMotor_1.setInverted(false);
    armMotor_2.setInverted(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    armCancoderPos = cancoder.getAbsolutePosition();

    armMotor1Pos = armMotor_1.getPosition();

    armMotor2Pos = armMotor_2.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, armMotor1Pos, armMotor2Pos, armCancoderPos);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, armMotor1Pos, armMotor2Pos, armCancoderPos);
    armMotor_1.optimizeBusUtilization();
    armMotor_2.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    BaseStatusSignal.refreshAll(armMotor1Pos, armMotor2Pos, armCancoderPos);

    inputs.armMotor1Pos = armMotor1Pos.getValueAsDouble();
    inputs.armMotor2Pos = armMotor2Pos.getValueAsDouble();
    inputs.armCancoderPos = armCancoderPos.getValueAsDouble();
  }

  @Override
  public void setArmMotor1Request(double position) {
    armMotor_1.setControl(new MotionMagicVoltage(position));
  }

  @Override
  public void setArmMotor2Follow() {
    // Comment this out to read encoder offsets
    armMotor_2.setControl(m_follow);
  }

  @Override
  public void setArmMotor1BrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    armMotor_1.getConfigurator().apply(config);
  }

  @Override
  public void setArmMotor2BrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    armMotor_2.getConfigurator().apply(config);
  }

  @Override
  public void setArmMotor1Pos(double value) {
    armMotor_1.setPosition(value);
  }

  @Override
  public void setArmMotor2Pos(double value) {
    armMotor_2.setPosition(value);
  }

  @Override
  public void setArmMotor1Speed(double speed) {
    armMotor_1.set(speed);
  }

  @Override
  public void setArmMotor2Speed(double speed) {
    armMotor_2.set(speed);
  }
}
