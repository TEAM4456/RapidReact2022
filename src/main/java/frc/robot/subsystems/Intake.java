// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.runIntake;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX intakeTalon1;
  private final WPI_TalonSRX intakeTalon2;
  private Timer timer;

  public Intake(WPI_TalonSRX intake1, WPI_TalonSRX intake2) {
    intakeTalon1 = intake1;
    intakeTalon2 = intake2;
  }

  public void runIntake(double percentOutput) {
    intakeTalon1.set(ControlMode.PercentOutput, percentOutput);
    intakeTalon2.set(ControlMode.PercentOutput, percentOutput);
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
