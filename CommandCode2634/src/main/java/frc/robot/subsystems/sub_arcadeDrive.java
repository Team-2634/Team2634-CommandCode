package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sub_arcadeDrive extends SubsystemBase {
  DifferentialDrive m_robotDrive = arcadeDriveTrain.getRobotDrive();

  public void arcadeDrive(double xSpeed, double zRotation)  {
    m_robotDrive.arcadeDrive(xSpeed, zRotation);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
