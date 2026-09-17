package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.intake.RollerConstants.HardwareConfig;

public class RollerSubsystem extends SubsystemBase {
  private final TalonFX motor;

  public RollerSubsystem() {
    motor = new TalonFX(HardwareConfig.kMotorId);

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    motor.getConfigurator().apply(config);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
  }

  public Command setSpeedCmd(double speed) {
    return runOnce(() -> motor.set(speed));
  }

  public void stop() {
    motor.stopMotor();
  }

  public Command stopCmd() {
    return runOnce(this::stop);
  }
}