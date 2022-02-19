// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private final WPI_TalonSRX armHeightTalon;
  private final WPI_TalonSRX armAngleTalon;
  /** Creates a new Arm. */
  public Arm(WPI_TalonSRX heightTalon, WPI_TalonSRX angleTalon) {
    armHeightTalon = heightTalon;
    armAngleTalon = angleTalon;
  }

  public void runHeight(double percentOutput) {
    armHeightTalon.set(ControlMode.PercentOutput, percentOutput);
  }

  public void runAngle(double percentOutput) {
    armAngleTalon.set(ControlMode.PercentOutput, percentOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
