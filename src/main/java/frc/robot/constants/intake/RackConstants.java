package frc.robot.constants.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public final class RackConstants {

  public static final class HardwareConfig {
    public static final int kMotorId = 17;
    public static final boolean kMotorInverted = true;
    public static final int kCurrentLimit = 40;

    public static final double kGearRatio = 5.0;

    /** Número de dentes da engrenagem. */
    public static final int kTeethCount = 10;

    /** Dentes por polegada (Diametral Pitch - DP). */
    public static final double kDiametralPitch = 10.0;

    /** Diâmetro primitivo da engrenagem em polegadas. (Teeth / DP) */
    private static final double kPitchDiameterInches = kTeethCount / kDiametralPitch;

    /** Circunferência primitiva da engrenagem em metros. */
    public static final double kPitchCircumferenceMeters = Units.inchesToMeters(kPitchDiameterInches * Math.PI);

    /** Fator de conversão de rotações do motor para metros lineares do rack. */
    public static final double kPositionConversionFactor = (1.0 / kGearRatio) * kPitchCircumferenceMeters;

    /** Fator de conversão de rotações do motor para metros por segundo. */
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

    /** Tolerância aceitável de erro */
    public static final Distance kDistanceTolerance = Meters.of(0.1);

    /** Limite de saída no modo lento */
    public static final double kSlowModeMaxOutput = 0.4;
  }

  public static final class Gains {
    public static final double kP = 0.0;
    public static final double kPSlow = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
  }

  public enum RackState {
    //POSITIONS
    INITIAL(0.0),
    INTAKE_READY(0.0),

    //AGITATE
    AGITATOR_FORWARD(0.0),
    AGITATOR_BACKWARD(0.0);

    public final Distance distance;

    RackState(double meters) {
      this.distance = Meters.of(meters);
    }
  }
}