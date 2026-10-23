package autopilot.core;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

import static org.junit.jupiter.api.Assertions.assertEquals;

class AdaptiveCruiseTest {

    private static final double TARGET_SPEED_KMH = 100.0;
    private static final double DELTA = 0.0001;

    @ParameterizedTest
    @ValueSource(doubles = {0.0, 40.0, 100.0})
    void safeDistanceFor_returnsHalfTheSpeed(double speedKmh) {
        AdaptiveCruise cruise = new AdaptiveCruise(TARGET_SPEED_KMH);

        double actualDistance = cruise.safeDistanceFor(speedKmh);

        assertEquals(speedKmh * 0.5, actualDistance, DELTA);
    }
}