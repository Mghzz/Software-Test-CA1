package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.params.provider.Arguments.arguments;

class GearBoxTest {

    private static final double STOPPED_SPEED_KMH = 0.0;
    private static final double REVERSE_LIMIT_BOUNDARY_SPEED_KMH = 5.0;
    private static final double ABOVE_REVERSE_LIMIT_SPEED_KMH = 10.0;
    private static final double NON_ZERO_BOUNDARY_SPEED_KMH = 0.1;
    private static final double MOVING_SPEED_KMH = 20.0;

    private double currentSpeedKmh;
    private GearBox gearBox;

    @BeforeEach
    void setUp() {
        currentSpeedKmh = STOPPED_SPEED_KMH;
        gearBox = new GearBox(() -> currentSpeedKmh);
    }

    @Test
    void changeTo_NullTargetGear_ThrowsNullPointerException() {
        assertThrows(
                NullPointerException.class,
                () -> gearBox.changeTo(null)
        );
    }

    @ParameterizedTest
    @EnumSource(GearBox.Gear.class)
    void changeTo_SameTargetGearAsCurrentGear_KeepsCurrentGear(GearBox.Gear gear) {
        transitionToGear(gear);
        currentSpeedKmh = MOVING_SPEED_KMH;

        gearBox.changeTo(gear);

        assertEquals(gear, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @ValueSource(doubles = {-0.1, -15.0, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY})
    void changeTo_InvalidSensorSpeed_ThrowsIllegalArgumentException(double invalidSpeed) {
        currentSpeedKmh = invalidSpeed;

        assertThrows(
                IllegalArgumentException.class,
                () -> gearBox.changeTo(GearBox.Gear.R)
        );
    }

    @ParameterizedTest
    @CsvFileSource(resources = "/valid-gear-transitions.csv", numLinesToSkip = 1)
    void changeTo_AllowedTransitionAndSpeed_UpdatesCurrentGear(
            GearBox.Gear fromGear,
            GearBox.Gear targetGear,
            double speedKmh
    ) {
        transitionToGear(fromGear);
        currentSpeedKmh = speedKmh;

        gearBox.changeTo(targetGear);

        assertEquals(targetGear, gearBox.getCurrentGear());
    }

    @ParameterizedTest
    @MethodSource("disallowedGearTransitionsProvider")
    void changeTo_DisallowedTransitionOrSpeed_ThrowsIllegalGearChangeException(
            GearBox.Gear fromGear,
            GearBox.Gear targetGear,
            double speedKmh
    ) {
        transitionToGear(fromGear);
        currentSpeedKmh = speedKmh;

        assertThrows(
                IllegalGearChangeException.class,
                () -> gearBox.changeTo(targetGear)
        );
    }

    static Stream<Arguments> disallowedGearTransitionsProvider() {
        return Stream.of(
                arguments(GearBox.Gear.P, GearBox.Gear.R, REVERSE_LIMIT_BOUNDARY_SPEED_KMH),
                arguments(GearBox.Gear.P, GearBox.Gear.R, ABOVE_REVERSE_LIMIT_SPEED_KMH),
                arguments(GearBox.Gear.R, GearBox.Gear.D, NON_ZERO_BOUNDARY_SPEED_KMH),
                arguments(GearBox.Gear.R, GearBox.Gear.D, MOVING_SPEED_KMH),
                arguments(GearBox.Gear.D, GearBox.Gear.R, NON_ZERO_BOUNDARY_SPEED_KMH),
                arguments(GearBox.Gear.D, GearBox.Gear.R, MOVING_SPEED_KMH),
                arguments(GearBox.Gear.P, GearBox.Gear.N, STOPPED_SPEED_KMH),
                arguments(GearBox.Gear.P, GearBox.Gear.D, STOPPED_SPEED_KMH),
                arguments(GearBox.Gear.R, GearBox.Gear.P, STOPPED_SPEED_KMH),
                arguments(GearBox.Gear.N, GearBox.Gear.P, STOPPED_SPEED_KMH),
                arguments(GearBox.Gear.N, GearBox.Gear.R, STOPPED_SPEED_KMH),
                arguments(GearBox.Gear.D, GearBox.Gear.P, STOPPED_SPEED_KMH),
                arguments(GearBox.Gear.D, GearBox.Gear.N, STOPPED_SPEED_KMH)
        );
    }

    private void transitionToGear(GearBox.Gear desiredGear) {
        currentSpeedKmh = STOPPED_SPEED_KMH;
        switch (desiredGear) {
            case P -> {
            }
            case R -> gearBox.changeTo(GearBox.Gear.R);
            case N -> {
                gearBox.changeTo(GearBox.Gear.R);
                gearBox.changeTo(GearBox.Gear.N);
            }
            case D -> {
                gearBox.changeTo(GearBox.Gear.R);
                gearBox.changeTo(GearBox.Gear.D);
            }
        }
    }
}
