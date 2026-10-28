package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.params.provider.Arguments.arguments;

@DisplayName("Auto Driver Revised Tests")
class AutoDriverRevisedTest {

    private static final double DEFAULT_SPEED_KMH = 80.0;
    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double SAFE_OBSTACLE_DISTANCE_METERS = 100.0;
    private static final double CENTERED_LANE_OFFSET_METERS = 0.0;
    private static final double ZERO_STEERING_ANGLE_DEGREES = 0.0;
    private static final double ZERO_BRAKE_PRESSURE = 0.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;
    private static final double DELTA = 0.0001;
    private static final String SENSOR_FAILURE_MESSAGE = "Sensor failed";

    private static final AutoDriver.DriveResult EXPECTED_NORMAL_HOLD_RESULT = new AutoDriver.DriveResult(
            ZERO_STEERING_ANGLE_DEGREES,
            ZERO_BRAKE_PRESSURE,
            AdaptiveCruise.Mode.HOLD,
            false,
            false
    );

    private static final AutoDriver.DriveResult EXPECTED_SAFE_MODE_RESULT = new AutoDriver.DriveResult(
            ZERO_STEERING_ANGLE_DEGREES,
            SAFE_MODE_BRAKE_PRESSURE,
            AdaptiveCruise.Mode.BRAKE,
            false,
            true
    );

    private static final SpeedSensor WORKING_SPEED_SENSOR = () -> DEFAULT_SPEED_KMH;
    private static final ObstacleSensor WORKING_OBSTACLE_SENSOR = () -> SAFE_OBSTACLE_DISTANCE_METERS;
    private static final LaneSensor WORKING_LANE_SENSOR = () -> CENTERED_LANE_OFFSET_METERS;

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = WORKING_SPEED_SENSOR;
        obstacleSensor = WORKING_OBSTACLE_SENSOR;
        laneSensor = WORKING_LANE_SENSOR;

        autoDriver = new AutoDriver(
                () -> speedSensor.getSpeedKmh(),
                () -> obstacleSensor.getDistanceMeters(),
                () -> laneSensor.getLaneOffsetMeters(),
                TARGET_SPEED_KMH
        );
    }

    @Test
    @DisplayName("tick() returns normal hold result when centered in lane with safe distance at target speed")
    void tick_CenteredLaneAndSafeObstacleDistanceAtTargetSpeed_ReturnsNormalHoldResult() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(EXPECTED_NORMAL_HOLD_RESULT, result);
    }

    @ParameterizedTest(name = "LaneOffset={0}m, ExpectedSteering={1}°")
    @DisplayName("tick() returns opposite steering correction angle when lane offset exceeds threshold")
    @CsvSource({
            "1.0,  -45.0",
            "-1.0,  45.0"
    })
    void tick_LaneOffsetExceedingThreshold_ReturnsOppositeSteeringCorrectionAngle(
            double laneOffsetMeters,
            double expectedSteeringAngleDegrees
    ) {
        laneSensor = () -> laneOffsetMeters;

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(expectedSteeringAngleDegrees, result.steeringAngleDegrees(), DELTA);
    }

    @ParameterizedTest(name = "Case #{index}: Sensor failure")
    @DisplayName("tick() returns safe mode result when a sensor failure occurs")
    @MethodSource("failingSensorsProvider")
    void tick_SensorFailureOccurs_ReturnsSafeModeResult(
            SpeedSensor candidateSpeedSensor,
            ObstacleSensor candidateObstacleSensor
    ) {
        speedSensor = candidateSpeedSensor;
        obstacleSensor = candidateObstacleSensor;

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertEquals(EXPECTED_SAFE_MODE_RESULT, result);
    }

    static Stream<Arguments> failingSensorsProvider() {
        SpeedSensor failingSpeedSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };
        ObstacleSensor failingObstacleSensor = () -> {
            throw new SensorFailureException(SENSOR_FAILURE_MESSAGE);
        };

        return Stream.of(
                arguments(failingSpeedSensor, WORKING_OBSTACLE_SENSOR),
                arguments(WORKING_SPEED_SENSOR, failingObstacleSensor)
        );
    }
}
