package autopilot.core;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.*;

@DisplayName("Steering Controller Tests")
class SteeringControllerTest {

    private static final double DELTA = 0.001;
    private static final double DEFAULT_SPEED = 50.0;
    private static final double MAX_CHANGE_PER_TICK = 60.0;

    private SteeringController controller;

    @BeforeEach
    void setUp() {
        controller = new SteeringController();
    }

    @ParameterizedTest(name = "Angle = {0}")
    @DisplayName("update() throws exception when requested angle is invalid")
    @ValueSource(doubles = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void update_InvalidAngle_ThrowsIllegalArgumentException(double invalidAngle) {
        assertThrows(IllegalArgumentException.class, () -> controller.update(invalidAngle, DEFAULT_SPEED));
    }

    @ParameterizedTest(name = "Speed = {0}")
    @DisplayName("update() throws exception when speed is invalid")
    @ValueSource(doubles = {-10.0, -0.1, Double.NaN, Double.POSITIVE_INFINITY})
    void update_InvalidSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> controller.update(30.0, invalidSpeed));
    }

    @ParameterizedTest(name = "Speed={0}km/h, Requested={1}°, ExpectedTarget={2}°, ExpectedCurrent={3}°")
    @DisplayName("update() clamps target angle and moves current angle incrementally based on speed limits")
    @CsvSource({
            "50.0, 400.0, 400.0, 60.0",
            "50.0, 500.0, 450.0, 60.0",
            "120.0, -600.0, -450.0, -60.0",
            "150.0, 50.0, 50.0, 50.0",
            "150.0, 200.0, 90.0, 60.0",
            "130.0, -100.0, -90.0, -60.0"
    })
    void update_ValidInputs_ClampsTargetAndMovesCurrent(
            double speed, double requestedAngle, double expectedTarget, double expectedCurrent) {

        double actualCurrent = controller.update(requestedAngle, speed);

        assertEquals(expectedTarget, controller.getTargetAngleDegrees(), DELTA);
        assertEquals(expectedCurrent, actualCurrent, DELTA);
        assertEquals(expectedCurrent, controller.getCurrentAngleDegrees(), DELTA);
    }

    @Test
    @DisplayName("update() moves incrementally when difference exceeds max tick limit")
    void update_DifferenceExceedsMaxTick_MovesIncrementally() {
        assertEquals(MAX_CHANGE_PER_TICK, controller.update(100.0, DEFAULT_SPEED), DELTA);
        assertEquals(100.0, controller.update(100.0, DEFAULT_SPEED), DELTA);
    }

    @Test
    @DisplayName("selfCenter() centers exactly when current angle is within max tick limit")
    void selfCenter_CurrentAngleWithinMaxTick_CentersExactly() {
        controller.update(45.0, DEFAULT_SPEED);
        double newAngle = controller.selfCenter();

        assertEquals(0.0, controller.getTargetAngleDegrees(), DELTA);
        assertEquals(0.0, newAngle, DELTA);
    }

    @Test
    @DisplayName("selfCenter() moves incrementally toward center when angle exceeds max tick limit")
    void selfCenter_CurrentAngleExceedsMaxTick_MovesIncrementallyTowardCenter() {
        controller.update(150.0, DEFAULT_SPEED);
        controller.update(150.0, DEFAULT_SPEED);

        double newAngle = controller.selfCenter();

        assertEquals(0.0, controller.getTargetAngleDegrees(), DELTA);
        assertEquals(60.0, newAngle, DELTA);
    }
}