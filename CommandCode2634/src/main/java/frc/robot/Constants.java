package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
    final XboxController m_Xstick = new XboxController(0);

    public final CANSparkMax leftFrontMax = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax rightFrontMax = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax leftBackMax = new CANSparkMax(17, MotorType.kBrushless);
    public final CANSparkMax rightBackMax = new CANSparkMax(18, MotorType.kBrushless);

    public final WPI_TalonFX leftFrontFX = new WPI_TalonFX(1);
    public final WPI_TalonFX rightFrontFX = new WPI_TalonFX(3);
    public final WPI_TalonFX leftBackFX = new WPI_TalonFX(2);
    public final WPI_TalonFX rightBackFX = new WPI_TalonFX(4);
}