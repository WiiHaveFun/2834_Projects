// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Drive extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  XboxController m_driverController;

  /** Creates a new Drive. */
  public Drive(DriveSubsystem driveSubsystem, XboxController controller) {
    m_driveSubsystem = driveSubsystem;
    m_driverController = controller;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double FWD = m_driverController.getRawAxis(1);
    if(Math.abs(FWD) < 0.2) {
      FWD = 0;
    } else {
      if(FWD > 0) {
        FWD = Math.pow(FWD, 2);
      } else {
        FWD = -Math.pow(FWD, 2);
      }
    }
    double STR = m_driverController.getRawAxis(0);
    if(Math.abs(STR) < 0.2) {
      STR = 0;
    } else {
      if(STR > 0) {
        STR = Math.pow(STR, 2);
      } else {
        STR = -Math.pow(STR, 2);
      }
    }
    double RCW = m_driverController.getRawAxis(2);
    if(Math.abs(RCW) < 0.2) {
      RCW = 0;
    }

    double speedMultiplier = 0.1;

    double[][] desiredStates = DriveSubsystem.calculate(-FWD * speedMultiplier, 
                                                        STR * speedMultiplier, 
                                                        RCW * speedMultiplier, 
                                                        m_driveSubsystem.getHeading(), 
                                                        DriveConstants.kWheelBase, 
                                                        DriveConstants.kTrackWidth);

    m_driveSubsystem.setModuleStates(desiredStates);
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
