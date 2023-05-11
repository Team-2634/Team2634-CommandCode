package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sub_arcadeDrive;

public class cmd_arcadeDrive extends CommandBase {
  private final XboxController xBox0;
  private final sub_arcadeDrive m_robotDrive;
  
  public cmd_arcadeDrive(sub_arcadeDrive robotDrive, XboxController Xstick) {
    xBox0 = Xstick;
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.arcadeDrive(xBox0.getRawAxis(4) *0.8, xBox0.getRawAxis(1) * 0.8);
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
