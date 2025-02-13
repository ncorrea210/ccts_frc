package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.GroundCoralConstants;

public class GroundCoralSubsystem extends SubsystemBase {

  public enum Setpoint {
    kIntake,
    kStow,
    kScore;
  }

  private SparkMax rotateMotor =
      new SparkMax(GroundCoralConstants.kRotateMotorID, MotorType.kBrushless);
  private SparkClosedLoopController rotateController = rotateMotor.getClosedLoopController();
  private AbsoluteEncoder rotateEncoder = rotateMotor.getAbsoluteEncoder();

  private SparkMax leftIntake =
      new SparkMax(GroundCoralConstants.kLeftIntakeID, MotorType.kBrushless);
  private SparkMax rightIntake =
      new SparkMax(GroundCoralConstants.kRightIntakeID, MotorType.kBrushless);

  public GroundCoralSubsystem() {
    rotateMotor.configure(
        Configs.GroundCoralSubsystem.rotateConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    leftIntake.configure(
        Configs.GroundCoralSubsystem.leftIntakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    rightIntake.configure(
        Configs.GroundCoralSubsystem.rightIntakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }
}
