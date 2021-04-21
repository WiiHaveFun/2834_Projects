// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BrushConstants;
import frc.robot.subsystems.Brush;

public class RunBrush extends CommandBase {
  Brush m_brush;
  XboxController m_controller;

  /** Creates a new RunBrush. */
  public RunBrush(Brush brush, XboxController controller) {
    m_brush = brush;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_brush);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  boolean unjamming = false;
  double startTime;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_controller.getYButton()) {
      m_brush.setPower(-BrushConstants.kNominalPower); 
    } else if(m_brush.getOutputCurrent() > 40.0) {
      if(!unjamming) {
        startTime = Timer.getFPGATimestamp();
        unjamming = true;
        m_brush.setPower(-BrushConstants.kNominalPower);
      }
    } else if (unjamming && Timer.getFPGATimestamp() - startTime > 0.3) {
        unjamming = false;
        m_brush.setPower(BrushConstants.kNominalPower);
    } else if(!unjamming) {
      m_brush.setPower(BrushConstants.kNominalPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_brush.setPower(-BrushConstants.kNeutralPower);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
