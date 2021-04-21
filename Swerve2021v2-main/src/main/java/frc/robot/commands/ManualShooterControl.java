// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ManualShooterControl extends CommandBase {
  Shooter m_shooter;
  XboxController m_controller;

  /** Creates a new ManualShooterControl. */
  public ManualShooterControl(Shooter shooter, XboxController controller) {
    m_shooter = shooter;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("Manual_Turret", false)) {
      // Flywheel control
      if(SmartDashboard.getNumber("Man_RPM", 0) != 0) {
        m_shooter.setFlySpeed(SmartDashboard.getNumber("Man_RPM", 0));
      } else {
        m_shooter.setFlyPower(0.0);
      }

      // Hood control
      if(!SmartDashboard.getBoolean("Manual_Hood_Set", false)) {
        if(m_controller.getPOV() == 0) {
          // Decrease angle/hood up
          m_shooter.moveHood(-1.0);
        } else if(m_controller.getPOV() == 180) {
          // Increase angle/hood down
          m_shooter.moveHood(1.0);
        } else {
          m_shooter.moveHood(0.0);
        }
      } else {
        m_shooter.setHoodPosition((int) SmartDashboard.getNumber("Man_Hood", 0));
      }

      // Turret control
      if(!SmartDashboard.getBoolean("Manual_Turret_Set", false)) {
        if(m_controller.getPOV() == 90) {
          m_shooter.moveTurret(-0.2);
        } else if(m_controller.getPOV() == 270) {
          m_shooter.moveTurret(0.2);
        } else {
          m_shooter.moveTurret(0.0);
        }
      } else {
        m_shooter.setTurretAngle(SmartDashboard.getNumber("Man_Turret", 0.0));
      }
    } else {
      m_shooter.setFlyPower(0.0);
      m_shooter.moveHood(0.0);
      m_shooter.moveTurret(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
