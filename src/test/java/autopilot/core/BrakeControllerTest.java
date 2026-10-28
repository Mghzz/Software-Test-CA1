package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

@DisplayName("Brake Controller Tests")
class BrakeControllerTest {

    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;
    private static final double ABS_TRIGGER_PRESSURE = 85.0;
    private static final double ABS_TRIGGER_SPEED_KMH = 50.0;
    private static final double NORMAL_PRESSURE = 50.0;
    private static final double NORMAL_SPEED_KMH = 20.0;
    private static final double DELTA = 0.0001;

    private BrakeController brakeController;

    @BeforeEach
    void setUp() {
        brakeController = new BrakeController();
    }

    @ParameterizedTest(name = "Pressure = {0}")
    @DisplayName("apply() throws exception when requested pressure is invalid")
    @ValueSource(doubles = {-0.1, -50.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void apply_InvalidRequestedPressure_ThrowsIllegalArgumentException(double invalidPressure) {
        assertThrows(
                IllegalArgumentException.class,
                () -> brakeController.apply(invalidPressure, NORMAL_SPEED_KMH, GearBox.Gear.D)
        );
    }

    @ParameterizedTest(name = "Speed = {0}")
    @DisplayName("apply() throws exception when speed is invalid")
    @ValueSource(doubles = {-0.1, -30.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void apply_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(
                IllegalArgumentException.class,
                () -> brakeController.apply(NORMAL_PRESSURE, invalidSpeed, GearBox.Gear.D)
        );
    }

    @Test
    @DisplayName("apply() throws exception when gear is null")
    void apply_NullGear_ThrowsNullPointerException() {
        assertThrows(
                NullPointerException.class,
                () -> brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED_KMH, null)
        );
    }

    @Test
    @DisplayName("apply() throws exception when gearbox is in park")
    void apply_GearInPark_ThrowsIllegalStateException() {
        assertThrows(
                IllegalStateException.class,
                () -> brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED_KMH, GearBox.Gear.P)
        );
    }

    @ParameterizedTest(name = "Requested={0}, Speed={1}km/h, Gear={2}, Expected={3}")
    @DisplayName("apply() returns bounded pressure when ABS conditions are not met")
    @CsvSource({
            "0.0,   0.0,  D, 0.0",
            "50.0,  25.0, D, 50.0",
            "80.0,  50.0, D, 80.0",
            "85.0,  30.0, R, 85.0",
            "100.0, 30.0, N, 100.0",
            "120.0, 20.0, D, 100.0"
    })
    void apply_NonAbsConditions_ReturnsBoundedPressure(
            double requestedPressure,
            double speedKmh,
            GearBox.Gear gear,
            double expectedPressure
    ) {
        double appliedPressure = brakeController.apply(requestedPressure, speedKmh, gear);

        assertEquals(expectedPressure, appliedPressure, DELTA);
    }

    @Test
    @DisplayName("apply() returns high ABS pulse on first call under ABS conditions")
    void apply_FirstCallUnderAbsConditions_ReturnsHighAbsPulse() {
        double appliedPressure = brakeController.apply(
                ABS_TRIGGER_PRESSURE,
                ABS_TRIGGER_SPEED_KMH,
                GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, appliedPressure, DELTA);
    }

    @Test
    @DisplayName("apply() alternates to low ABS pulse on second consecutive call under ABS conditions")
    void apply_SecondConsecutiveCallUnderAbsConditions_ReturnsLowAbsPulse() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        double secondPulsePressure = brakeController.apply(
                ABS_TRIGGER_PRESSURE,
                ABS_TRIGGER_SPEED_KMH,
                GearBox.Gear.D
        );

        assertEquals(ABS_LOW_PULSE, secondPulsePressure, DELTA);
    }

    @Test
    @DisplayName("apply() alternates back to high ABS pulse on third consecutive call under ABS conditions")
    void apply_ThirdConsecutiveCallUnderAbsConditions_ReturnsHighAbsPulse() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);

        double thirdPulsePressure = brakeController.apply(
                ABS_TRIGGER_PRESSURE,
                ABS_TRIGGER_SPEED_KMH,
                GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, thirdPulsePressure, DELTA);
    }

    @Test
    @DisplayName("apply() resets ABS pulse to high after a non-ABS braking call")
    void apply_AbsConditionsAfterNonAbsCall_ResetsToHighAbsPulse() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        brakeController.apply(NORMAL_PRESSURE, NORMAL_SPEED_KMH, GearBox.Gear.D);

        double appliedPressure = brakeController.apply(
                ABS_TRIGGER_PRESSURE,
                ABS_TRIGGER_SPEED_KMH,
                GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, appliedPressure, DELTA);
    }

    @Test
    @DisplayName("apply() resets ABS pulse to high after emergency braking")
    void apply_AbsConditionsAfterEmergencyBrake_ResetsToHighAbsPulse() {
        brakeController.apply(ABS_TRIGGER_PRESSURE, ABS_TRIGGER_SPEED_KMH, GearBox.Gear.D);
        brakeController.emergencyBrake(GearBox.Gear.D);

        double appliedPressure = brakeController.apply(
                ABS_TRIGGER_PRESSURE,
                ABS_TRIGGER_SPEED_KMH,
                GearBox.Gear.D
        );

        assertEquals(ABS_HIGH_PULSE, appliedPressure, DELTA);
    }

    @Test
    @DisplayName("emergencyBrake() throws exception when gear is null")
    void emergencyBrake_NullGear_ThrowsNullPointerException() {
        assertThrows(
                NullPointerException.class,
                () -> brakeController.emergencyBrake(null)
        );
    }

    @Test
    @DisplayName("emergencyBrake() throws exception when gearbox is in park")
    void emergencyBrake_GearInPark_ThrowsIllegalStateException() {
        assertThrows(
                IllegalStateException.class,
                () -> brakeController.emergencyBrake(GearBox.Gear.P)
        );
    }

    @ParameterizedTest(name = "Gear = {0}")
    @DisplayName("emergencyBrake() sets pressure to maximum for allowed gears")
    @EnumSource(value = GearBox.Gear.class, mode = EnumSource.Mode.EXCLUDE, names = {"P"})
    void emergencyBrake_AllowedGear_SetsMaxPressure(GearBox.Gear allowedGear) {
        brakeController.emergencyBrake(allowedGear);

        assertEquals(MAX_PRESSURE, brakeController.getPressure(), DELTA);
    }

    @ParameterizedTest(name = "Gear = {0}")
    @DisplayName("emergencyBrake() activates warning for allowed gears")
    @EnumSource(value = GearBox.Gear.class, mode = EnumSource.Mode.EXCLUDE, names = {"P"})
    void emergencyBrake_AllowedGear_ActivatesWarning(GearBox.Gear allowedGear) {
        brakeController.emergencyBrake(allowedGear);

        assertTrue(brakeController.isWarningActive());
    }
}
