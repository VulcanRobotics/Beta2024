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

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Queue;

/** 1218's custom IO implementation for the NavX gyro used in our robots * */
public class GyroIONavX implements GyroIO {
  private final AHRS navX = new AHRS(SPI.Port.kMXP);
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  private double lastYaw;

  public GyroIONavX() {
    navX.zeroYaw();
    lastYaw = 0.0;

    /*  Here we create a queue in using the SparkMaxOdometryThread. Even though we're
    not using SparkMax motors for the robot drive, it's a much easier object to
    use than the PhoenixOdometryThread, which uses a bunch of custom template classes.

    As far as I (YK) can tell, it's fine to have both the SparkMaxOdometry and
    PhoenixOdometry thread objects running simultaneously. The yaw readings are
    stored in yawPositionQueue and then streamed to the GyroIOInputs in updateInputs(). */
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(() -> -navX.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    float currentYaw =
        -navX.getYaw(); // NavX yaw direction (+CW) is opposite WPILib standard (+CCW)
    inputs.connected = navX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(currentYaw);
    SmartDashboard.putNumber("Gyro Yaw", currentYaw);

    /*  As far as I (YK) can tell, yaw velocity isn't used for any odometry calculations,
    only for logging/debugging. Since the NavX doesn't provide yaw velocity itsef,
    we take the difference between the current and last yaw values and divide
    by the time between readings (should be 1/250 seconds, or 250 Hz). */
    inputs.yawVelocityRadPerSec =
        Units.degreesToRadians((currentYaw - lastYaw) / Module.ODOMETRY_FREQUENCY);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear(); // Clear out our thread queue

    lastYaw = currentYaw; // Save the current yaw value; now it's lastYaw
  }

  @Override
  public void zeroGyro() {
    navX.zeroYaw();
  }
}
