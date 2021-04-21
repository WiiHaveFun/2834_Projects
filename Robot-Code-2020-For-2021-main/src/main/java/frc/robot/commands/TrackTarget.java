// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class TrackTarget extends CommandBase {
  // The subsystem the command runs on
  Shooter shooter;

  /** Creates a new TrackTarget. */
  public TrackTarget(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Aim only the turret toward the target
    // Check if the target is detected
    if(SmartDashboard.getBoolean("Target Detected?", false)) {
      // Point the turret at the target 
      double ticksToTarget = shooter.getTurretYawTick(SmartDashboard.getNumber("turretYawError", 0.0) + (Math.PI / 90), Constants.turretTicksPerRevolution);
      double turretTargetTick = shooter.turretMotor.getSelectedSensorPosition() + ticksToTarget;
      if((turretTargetTick >= Constants.turretLowLimitTick) && (turretTargetTick <= Constants.turretHighLimitTick)) {
        shooter.turretMotor.set(ControlMode.MotionMagic, shooter.turretMotor.getSelectedSensorPosition() + ticksToTarget);
      } else {
        shooter.turretMotor.set(ControlMode.MotionMagic, shooter.turretMotor.getSelectedSensorPosition() + ticksToTarget - Constants.turretTicksPerRevolution);
      }

      // Set tracking status to true
      SmartDashboard.putBoolean("Tracking?", true);
    } else {
      // Set tracking status to false
      SmartDashboard.putBoolean("Tracking?", false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.turretMotor.set(ControlMode.PercentOutput, 0.0);

    // Set tracking status to false
    SmartDashboard.putBoolean("Tracking?", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
