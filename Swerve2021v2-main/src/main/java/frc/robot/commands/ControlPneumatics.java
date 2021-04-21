// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class ControlPneumatics extends CommandBase {
  Pneumatics m_pneumatics;
  Boolean output = null;

  /** Creates a new ControlPneumatics. */
  public ControlPneumatics(Pneumatics pneumatics, boolean output) {
    m_pneumatics = pneumatics;
    this.output = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pneumatics);
  }

  /** Creates a new ControlPneumatics. 
   * For toggling position.
  */
  public ControlPneumatics(Pneumatics pneumatics) {
    m_pneumatics = pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(output == null) {
      m_pneumatics.setSolenoidOutput(!m_pneumatics.getSolenoidOutput());
    } else {
      m_pneumatics.setSolenoidOutput(output);
    }    
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
