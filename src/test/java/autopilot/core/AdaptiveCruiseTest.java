package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

@DisplayName("Adaptive Cruise Tests")
class AdaptiveCruiseTest {

    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double MAX_ACCELERATION_STEP = 5.0;
    private static final double HYSTERESIS_BRAKE = 10.0;

    private AdaptiveCruise cruise;

    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(TARGET_SPEED_KMH);
    }

    @ParameterizedTest(name = "Target Speed = {0}")
    @DisplayName("constructor throws exception on invalid target speed")
    @ValueSource(doubles = {-10.0, -0.1, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void constructor_InvalidTargetSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> new AdaptiveCruise(invalidSpeed));
    }

    @ParameterizedTest(name = "Speed = {0}")
    @DisplayName("safeDistanceFor() throws exception on invalid speed")
    @ValueSource(doubles = {-5.0, Double.NaN, Double.POSITIVE_INFINITY})
    void safeDistanceFor_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(invalidSpeed));
    }

    @ParameterizedTest(name = "Speed={0}km/h, Expected Distance={1}m")
    @DisplayName("safeDistanceFor() calculates valid safe distance based on speed")
    @CsvSource({
            "100.0, 50.0",
            "0.0, 0.0",
            "60.0, 30.0",
            "120.5, 60.25"
    })
    void safeDistanceFor_ValidSpeed_ReturnsCorrectDistance(double speedKmh, double expectedDistance) {
        assertEquals(expectedDistance, cruise.safeDistanceFor(speedKmh));
    }

    @Test
    @DisplayName("update() triggers emergency braking when distance is critically low")
    void update_DistanceUnderEmergency_ReturnsEmergencyBrakeDecision() {
        AdaptiveCruise.Decision decision = cruise.update(TARGET_SPEED_KMH, 20.0);

        assertEquals(AdaptiveCruise.Mode.EMERGENCY_BRAKE, decision.mode());
        assertEquals(100.0, decision.amount());
    }

    @Test
    @DisplayName("update() triggers proportional braking when distance is unsafe")
    void update_DistanceUnderSafe_ReturnsProportionalBrakeDecision() {
        AdaptiveCruise.Decision decision = cruise.update(TARGET_SPEED_KMH, 40.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(20.0, decision.amount());
    }

    @Test
    @DisplayName("update() accelerates up to maximum step when safe and below target speed")
    void update_DistanceOverSafeSpeedUnderTarget_ReturnsAccelerateDecision() {
        AdaptiveCruise.Decision decision = cruise.update(80.0, 60.0);

        assertEquals(AdaptiveCruise.Mode.ACCELERATE, decision.mode());
        assertEquals(MAX_ACCELERATION_STEP, decision.amount());
    }

    @Test
    @DisplayName("update() holds speed when safe and precisely at target speed")
    void update_DistanceOverSafeSpeedAtTarget_ReturnsHoldDecision() {
        AdaptiveCruise.Decision decision = cruise.update(TARGET_SPEED_KMH, 100.0);

        assertEquals(AdaptiveCruise.Mode.HOLD, decision.mode());
        assertEquals(0.0, decision.amount());
    }

    @Test
    @DisplayName("update() applies hysteresis braking when transitioning between safe and release distances")
    void update_BrakingHysteresisActive_ReturnsHysteresisBrakeDecision() {
        cruise.update(TARGET_SPEED_KMH, 40.0); // Force braking mode
        AdaptiveCruise.Decision decision = cruise.update(TARGET_SPEED_KMH, 55.0);

        assertEquals(AdaptiveCruise.Mode.BRAKE, decision.mode());
        assertEquals(HYSTERESIS_BRAKE, decision.amount());
    }
}