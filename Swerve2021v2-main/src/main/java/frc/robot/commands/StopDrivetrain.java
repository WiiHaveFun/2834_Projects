// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class StopDrivetrain extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  
  /** 
   * Creates a new StopDrivetrain. 
   * 
   * @param driveSubsystem The drive subsystem of the robot.
   * */
  public StopDrivetrain(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setModuleStates(DriveSubsystem.calculate(0.0, 
                                                              0.0, 
                                                              0.0, 
                                                              m_driveSubsystem.getHeading(), 
                                                              DriveConstants.kWheelBase, 
                                                              DriveConstants.kTrackWidth));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
