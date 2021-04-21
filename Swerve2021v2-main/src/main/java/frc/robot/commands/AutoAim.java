// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoAim extends CommandBase {
  Shooter m_shooter;
  boolean aimOnly;

  /** Creates a new AutoAim. */
  public AutoAim(Shooter shooter, boolean aimOnly) {
    m_shooter = shooter;
    this.aimOnly = aimOnly;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!aimOnly) {
      m_shooter.setHoodPosition((int) SmartDashboard.getNumber("targetAngleAve", 0.0));
      m_shooter.setFlySpeed(SmartDashboard.getNumber("targetRPMAve", 0.0));
    }
    if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 1.0) {
      double deltaDeg = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0) + 1.0;
      m_shooter.setTurretAngle(m_shooter.getTurretAngle() - deltaDeg);
    } else {
      m_shooter.moveTurret(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.moveHood(0.0);
    m_shooter.setFlyPower(0.0);
    m_shooter.moveTurret(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
