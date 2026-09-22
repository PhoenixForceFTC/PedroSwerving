/*
 * Copyright (c) 2026 Pedro Pathing
 * SPDX-License-Identifier: BSD-3-Clause
 */
package com.pedropathing.integration;

import static org.junit.jupiter.api.Assertions.*;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Twist;
import com.pedropathing.math.Vector2D;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import java.util.Collections;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Contracts relied on by Phoenix's robot-owned drivetrain and guarded follower. */
class PhoenixContractTest {
    @Test
    void fieldVelocityBecomesBodyTwistAtQuarterTurn() {
        MotionState state = MotionState.ofVelocity(new Pose(0, 0, Math.PI / 2), new Velocity(-3, 4, .5));
        assertEquals(4, state.twist().vx, 1e-9);
        assertEquals(3, state.twist().vy, 1e-9);
        assertEquals(.5, state.twist().omega, 0);
    }

    @Test
    void manualPowersStayRobotRelative() {
        Probe p = new Probe();
        p.setPose(new Pose(0, 0, Math.PI / 2));
        Follower f = new Follower(p, p, algorithm());
        f.manual(.2, -.1, .05);
        f.update(.02);
        assertEquals(.2, p.last.forward(), 0);
        assertEquals(-.1, p.last.strafe(), 0);
        assertEquals(.05, p.last.turn(), 0);
        assertEquals(1, p.reads);
    }

    @Test
    void allocatorConvertsWorldTranslationExactlyOnce() {
        Foresight a = algorithm();
        MotionState state = MotionState.ofVelocity(new Pose(0, 0, Math.PI / 2), Velocity.zero());
        DrivePowers powers = a.allocator.getDrivePowers(Vector2D.cartesian(0, .5), state, .1);
        assertEquals(.5, powers.forward(), 1e-9);
        assertEquals(0, powers.strafe(), 1e-9);
        assertEquals(.1, powers.turn(), 0);
    }

    @Test
    void lineAndCurvePreserveEndpointAndHeading() {
        Pose start = new Pose(0, 0, .2), end = new Pose(12, 8, .8);
        Path line = Paths.line(start, end).linear(start, end);
        Path curve = Paths.curve(start, new Pose(6, 6, 0), end).constant(start);
        assertEquals(12, line.endPose().x(), 1e-9);
        assertEquals(.8, line.endPose().heading(), 1e-9);
        assertEquals(8, curve.endPose().y(), 1e-9);
        assertEquals(.2, curve.endPose().heading(), 1e-9);
    }

    @Test
    void syntheticBrakingModelIsFiniteAndOpposesBothDirections() {
        Foresight a = algorithm();
        for (double speed : new double[] {-8, 0, 8}) {
            Pose displacement = a.getBrakeDisplacement(new Twist(speed, 0, 0), 0);
            assertEquals(Math.signum(speed), Math.signum(displacement.x()), 0);
            double velocity = a.getVelocityToBrakeInTime(displacement.x(), Vector2D.cartesian(1, 0), 0);
            assertEquals(speed, velocity, 1e-9);
        }
    }

    @Test
    void publishedStopNeedsRobotWrapperForImmediateHardwareStop() {
        Probe p = new Probe();
        Follower f = new Follower(p, p, algorithm());
        f.manual(.2, 0, 0);
        f.update(.02);
        f.stop();
        assertTrue(f.idle());
        assertEquals(0, p.stops); // Deliberate 3.0.1 contract; SafePedroFollower closes this gap.
        f.update(.02);
        assertEquals(1, p.stops);
    }

    @Test
    void publishedFollowCompletesParametricallyBeforeHeadingAccuracy() {
        Probe p = new Probe();
        Follower f = new Follower(p, p, algorithm());
        f.holdEnd.set(false);
        f.follow(Paths.line(Pose.zero(), new Pose(12, 0, 0)).constant(0));
        p.setPose(new Pose(12, 0, 1));
        f.update(.02);
        f.update(.02);
        assertTrue(f.idle()); // Robot diagnostics must assess terminal pose independently.
        assertEquals(1, p.pose().heading(), 0);
    }

    private static Foresight algorithm() {
        return new Foresight(new ForesightConfig(c -> {
            c.headingFeedback.set(Controller.proportional(.5));
            c.forwardTranslational.set(Controller.proportional(.03));
            c.strafeTranslational.set(Controller.proportional(.03));
            c.brake.set(Controller.proportionalFeedforward(.1));
            c.coast.set(Controller.proportionalFeedforward(.1));
            c.linearBrakeCoefficients.set(Matrix.diag(.08, .09));
            c.quadraticBrakeCoefficients.set(Matrix.diag(.01, .012));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(.1, .02));
            c.maxAchievableForwardVelocity.set(12.0);
            c.maxAchievableStrafeVelocity.set(10.0);
            c.naturalForwardDeceleration.set(9.0);
            c.naturalStrafeDeceleration.set(8.0);
            c.pathSkip.set(false);
        }));
    }

    private static final class Probe implements Localizer, Drivetrain {
        MotionState state = MotionState.zero();
        DrivePowers last = DrivePowers.zero();
        int reads, stops;

        @Override
        public void update() {
            reads++;
        }

        @Override
        public void reset() {
            state = MotionState.zero();
        }

        @Override
        public MotionState state() {
            return state;
        }

        @Override
        public void setPose(Pose pose) {
            state = state.withPose(pose);
        }

        @Override
        public void drive(DrivePowers powers, boolean manual) {
            last = powers;
        }

        @Override
        public void stop() {
            stops++;
            last = DrivePowers.zero();
        }

        @Override
        public void stop(boolean brake) {
            stop();
        }

        @Override
        public double maxScaling(DrivePowers current, DrivePowers delta) {
            return 1;
        }

        @Override
        public double interpolateVelocity(double x, double y, double theta) {
            return x;
        }

        @Override
        public Map<String, Object> debug() {
            return Collections.emptyMap();
        }
    }
}
