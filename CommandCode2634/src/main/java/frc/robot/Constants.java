
package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
    final XboxController m_Xstick = new XboxController(0);

    public final CANSparkMax leftFrontMax = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax rightFrontMax = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax leftBackMax = new CANSparkMax(17, MotorType.kBrushless);
    public final CANSparkMax rightBackMax = new CANSparkMax(18, MotorType.kBrushless);
}