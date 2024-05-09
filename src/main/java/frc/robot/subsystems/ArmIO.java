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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

  @AutoLog
  public static class ArmIOInputs {
    public double armMotor1Pos = 0.0;
    public double armMotor2Pos = 0.0;
    public double armCancoderPos = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ArmIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setArmMotor1Request(MotionMagicVoltage request) {}

  /** Run the turn motor at the specified voltage. */
  public default void setArmMotor2Follow(Follower follow) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setArmMotor1BrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setArmMotor2BrakeMode(boolean enable) {}

  /** Sets motor1 encoder position to the desired value. */
  public default void setArmMotor1Pos(double value) {}

  /** Sets motor2 encoder position to the desired value. */
  public default void setArmMotor2Pos(double value) {}

  /** Run motor1 at a speed output. */
  public default void setArmMotor1Speed(double speed) {}

  /** Run motor2 at a speed output. */
  public default void setArmMotor2Speed(double speed) {}
}
