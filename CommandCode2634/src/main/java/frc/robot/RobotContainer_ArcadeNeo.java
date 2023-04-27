
package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.com_arcadeDrive;
import frc.robot.subsystems.arcadeDriveTrain;
import frc.robot.subsystems.sub_arcadeDrive;

public class RobotContainer_ArcadeNeo {
  final Constants cont = new Constants();
  final arcadeDriveTrain aTrain = new arcadeDriveTrain(cont.leftFrontMax, cont.rightFrontMax, cont.leftBackMax, cont.rightBackMax);
  private final sub_arcadeDrive m_robotArcadeDrive = new sub_arcadeDrive();
  private final com_arcadeDrive arcadeDrivingCom = new com_arcadeDrive(m_robotArcadeDrive, cont.m_Xstick);

  public RobotContainer_ArcadeNeo() {
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
