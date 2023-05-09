package frc.robot;

import frc.robot.commands.com_arcadeDrive;
import frc.robot.subsystems.arcadeDriveTrain;
import frc.robot.subsystems.sub_arcadeDrive;

public class RobotContainer {
  final Constants cont = new Constants();
  private final arcadeDriveTrain arcadeDriveTrain_Neo = new arcadeDriveTrain(cont.leftFrontMax, cont.rightFrontMax, cont.leftBackMax, cont.rightBackMax);
  //private final arcadeDriveTrain arcadeDriveTrain_Talon = new arcadeDriveTrain(cont.leftFrontFX, cont.rightFrontFX, cont.leftBackFX, cont.rightBackFX);
  private final sub_arcadeDrive m_robotArcadeDrive = new sub_arcadeDrive();
  private final com_arcadeDrive arcadeDrivingCom = new com_arcadeDrive(m_robotArcadeDrive, cont.xBox0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_robotArcadeDrive.setDefaultCommand(arcadeDrivingCom);
  }

  /*
  public Command getAutonomousCommand() {
  }
   */
}