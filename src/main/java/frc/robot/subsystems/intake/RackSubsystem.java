package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.intake.RackConstants.Gains;
import frc.robot.constants.intake.RackConstants.HardwareConfig;
import frc.robot.constants.intake.RackConstants.RackState;

public class RackSubsystem extends SubsystemBase {

  private final SparkMax motor = new SparkMax(HardwareConfig.kMotorId, MotorType.kBrushless);
  private final SparkMaxConfig motorConfig = new SparkMaxConfig();
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private Distance targetDistance = RackState.INITIAL.distance;

  // PID apenas para tuning via Dashboard
  private final PIDController tuningPid = new PIDController(Gains.kP, Gains.kI, Gains.kD);
  private double currentKp = Gains.kP;
  private double currentKi = Gains.kI;
  private double currentKd = Gains.kD;

  public RackSubsystem() {
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    configureMotor();

    SmartDashboard.putData("Rack/PID Tuning", tuningPid);
  }

  private void configureMotor() {
    motorConfig
        .idleMode(IdleMode.kBrake)
        .inverted(HardwareConfig.kMotorInverted)
        .smartCurrentLimit(HardwareConfig.kCurrentLimit);

    motorConfig.encoder
        .positionConversionFactor(HardwareConfig.kPositionConversionFactor)
        .velocityConversionFactor(HardwareConfig.kVelocityConversionFactor);

    motorConfig.closedLoop
        // Slot 0: velocidade normal
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(Gains.kP)
        .i(Gains.kI)
        .d(Gains.kD)

        // Slot 1: velocidade reduzida (agitação/chacoalhar)
        .p(Gains.kPSlow, ClosedLoopSlot.kSlot1)
        .i(Gains.kI, ClosedLoopSlot.kSlot1)
        .d(Gains.kD, ClosedLoopSlot.kSlot1)
        .outputRange(-HardwareConfig.kSlowModeMaxOutput, HardwareConfig.kSlowModeMaxOutput, ClosedLoopSlot.kSlot1);

    encoder.setPosition(RackState.INITIAL.distance.in(Meters));
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    updateTunablePID();
    updateTelemetry();
  }

  private void updateTelemetry() {
    SmartDashboard.putNumber("Rack/Current Pos (m)", getPosition().in(Meters));
    SmartDashboard.putNumber("Rack/Setpoint (m)", targetDistance.in(Meters));
    SmartDashboard.putBoolean("Rack/At Target", isAtTarget());
  }

  public Distance getPosition() {
    return Meters.of(encoder.getPosition());
  }

  public boolean isAtTarget() {
    return Math.abs(getPosition().minus(targetDistance).in(Meters)) < HardwareConfig.kDistanceTolerance
        .in(Meters);
  }

  public void setPosition(RackState state) {
    targetDistance = state.distance;

    closedLoopController.setSetpoint(
        targetDistance.in(Meters),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0);
  }

  public Command setPositionCmd(RackState position) {
    return runOnce(() -> setPosition(position));
  }

  public void setPositionSlow(RackState position) {
    targetDistance = position.distance;

    closedLoopController.setSetpoint(
        targetDistance.in(Meters),
        ControlType.kPosition,
        ClosedLoopSlot.kSlot1);
  }

  public Command setPositionSlowCmd(RackState position) {
    return runOnce(() -> setPosition(position));
  }

  public void stop() {
    motor.stopMotor();
  }

  private void updateTunablePID() {
    double newP = tuningPid.getP();
    double newI = tuningPid.getI();
    double newD = tuningPid.getD();

    if (newP != currentKp || newI != currentKi || newD != currentKd) {
      currentKp = newP;
      currentKi = newI;
      currentKd = newD;

      // Manter comentado, apenas usar para ajuste do PID durante testes
      // motorConfig.closedLoop.p(currentKp).i(currentKi).d(currentKd);
      // motor.configure(motorConfig, ResetMode.kNoResetSafeParameters,
      // PersistMode.kNoPersistParameters);
    }
  }
}