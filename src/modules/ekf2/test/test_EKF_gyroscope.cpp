/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Gyro-related tests (bias convergence, ...)
 */

#include <gtest/gtest.h>
#include <string>
#include <tuple>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfGyroscopeTest : public ::testing::Test
{
public:

	EkfGyroscopeTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// Init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}

	void testBias(const Vector3f &bias, float duration, const Vector3f &tolerance);
};

void EkfGyroscopeTest::testBias(const Vector3f &bias, float duration, const Vector3f &tolerance)
{
	_sensor_simulator._imu.setGyroData(bias);
	_sensor_simulator.runSeconds(duration);
	EXPECT_TRUE(_ekf->control_status_flags().vehicle_at_rest);

	const Vector3f estimated_bias = _ekf->getGyroBias();

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(estimated_bias(i), bias(i), tolerance(i)) << "index " << i;
	}
}

TEST_F(EkfGyroscopeTest, biasEstimateZero)
{
	testBias(Vector3f(), 10, Vector3f());
}

TEST_F(EkfGyroscopeTest, biasEstimatePositive)
{
	// The estimate should track a slowly changing bias
	const float biases[4] = {0.001f, 0.002f, 0.0035f, 0.005f};
	Vector3f bias;

	for (int i = 0; i < 4; i ++) {
		bias.setAll(biases[i]);
		// The Z gyro bias takes more time to converge as the Z rotation variance is higher
		testBias(bias, 30, Vector3f(0.0008f, 0.0008f, 0.004f));
	}
}

TEST_F(EkfGyroscopeTest, biasEstimateNegative)
{
	const float biases[4] = {-0.001f, -0.002f, -0.0035, -0.005f};
	Vector3f bias;

	for (int i = 0; i < 4; i ++) {
		bias.setAll(biases[i]);
		testBias(bias, 30, Vector3f(0.0008f, 0.0008f, 0.004f));
	}
}

// Feed the actual EKF synthetic Earth rotation plus a known sensor bias. The
// sensor simulator does not add Earth rotation on its own. INIT-only compass
// fusion prevents a continuous heading reference from concealing gyro drift.
class EkfEarthRateGyroscopeTest : public EkfGyroscopeTest,
	public ::testing::WithParamInterface<std::tuple<float, float, float, float>>
{
public:
	Vector3f earthRateBody(const Quatf &attitude) const
	{
		const float latitude = radians(std::get<0>(GetParam()));
		const Vector3f earth_rate{CONSTANTS_EARTH_SPIN_RATE * cosf(latitude), 0.f,
					  -CONSTANTS_EARTH_SPIN_RATE * sinf(latitude)};
		return attitude.rotateVectorInverse(earth_rate);
	}

	Quatf referenceAttitude(float yaw_offset = 0.f) const
	{
		return Quatf(Eulerf(radians(std::get<1>(GetParam())), radians(std::get<2>(GetParam())),
				    radians(std::get<3>(GetParam())) + yaw_offset));
	}

	void initializeEarthRate()
	{
		_ekf->getParamHandle()->mag_declination_source = 0;
		_ekf->getParamHandle()->mag_declination_deg = 0.f;
		_ekf->getParamHandle()->mag_fusion_type = MagFuseType::INIT;
		_sensor_simulator.simulateOrientation(referenceAttitude());
		_sensor_simulator._imu.setGyroData(earthRateBody(referenceAttitude()) + _bias);
		_sensor_simulator.runSeconds(10.f);
		ASSERT_TRUE(_ekf->control_status_flags().yaw_align);
		ASSERT_TRUE(_ekf->resetGlobalPositionTo(std::get<0>(GetParam()), -79.9462941, 122.2f));
		ASSERT_TRUE(_ekf->setEkfGlobalOrigin(std::get<0>(GetParam()), -79.9462941, 122.2f));
		// Allow stationary bias observations to settle after latitude becomes
		// available, including the higher Earth-rate component at high latitude.
		_sensor_simulator.runSeconds(300.f);
		ASSERT_TRUE(_ekf->control_status_flags().vehicle_at_rest);
		ASSERT_FALSE(_ekf->control_status_flags().mag_hdg);
		ASSERT_FALSE(_ekf->control_status_flags().mag_3D);
	}

	const Vector3f _bias{0.0004f, -0.0003f, 0.0002f};
};

TEST_P(EkfEarthRateGyroscopeTest, learnsSensorBiasWithoutAbsorbingEarthRotation)
{
	initializeEarthRate();
	ASSERT_FALSE(HasFatalFailure());

	for (unsigned axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(_ekf->getGyroBias()(axis), _bias(axis), 2e-6f) << "axis " << axis;
	}

	const float yaw_at_rest = _ekf_wrapper.getYawAngle();
	_sensor_simulator.runSeconds(300.f);
	RecordProperty("stationary_drift_deg", std::to_string(degrees(wrap_pi(_ekf_wrapper.getYawAngle() - yaw_at_rest))));
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - yaw_at_rest), 0.f, radians(0.03f));

	// Stop stationary bias observations, as when the aircraft starts moving.
	// The previously learned bias must remain usable without an absolute yaw aid.
	_ekf->set_vehicle_at_rest(false);
	const float yaw_when_moving = _ekf_wrapper.getYawAngle();
	_sensor_simulator.runSeconds(300.f);
	EXPECT_FALSE(_ekf->control_status_flags().vehicle_at_rest);
	RecordProperty("nonstationary_drift_deg",
		       std::to_string(degrees(wrap_pi(_ekf_wrapper.getYawAngle() - yaw_when_moving))));
	EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - yaw_when_moving), 0.f, radians(0.03f));
}

TEST_P(EkfEarthRateGyroscopeTest, tracksFullTurnsBothDirectionsWithoutCompassYawFusion)
{
	initializeEarthRate();
	ASSERT_FALSE(HasFatalFailure());
	_ekf->set_vehicle_at_rest(false);
	const float starting_yaw = _ekf_wrapper.getYawAngle();
	const auto reset_count = _ekf_wrapper.getQuaternionResetCounter();
	float physical_yaw_change = 0.f;

	// Four 90-degree turns in each direction, crossing the +/-pi boundary.
	// Body rates include the full 3-D projection of a turn about earth-down;
	// this also exercises non-level attitudes, not only the Z gyroscope.
	for (const float direction : {1.f, -1.f}) {
		const float yaw_rate = direction * radians(90.f);

		for (unsigned quarter_turn = 0; quarter_turn < 4; ++quarter_turn) {
			for (unsigned sample = 0; sample < 50; ++sample) {
				const Quatf attitude = referenceAttitude(physical_yaw_change + yaw_rate * 0.01f);
				_sensor_simulator.simulateOrientation(attitude);
				_sensor_simulator._imu.setGyroData(earthRateBody(attitude) + _bias
								   + attitude.rotateVectorInverse(Vector3f{0.f, 0.f, yaw_rate}));
				_sensor_simulator.runMicroseconds(20000);
				physical_yaw_change += yaw_rate * 0.02f;
			}

			// The EKF has a delayed fusion horizon: allow the completed turn to
			// reach it before comparing against the physical end orientation.
			const Quatf end_attitude = referenceAttitude(physical_yaw_change);
			_sensor_simulator.simulateOrientation(end_attitude);
			_sensor_simulator._imu.setGyroData(earthRateBody(end_attitude) + _bias);
			_sensor_simulator.runSeconds(0.5f);
			EXPECT_NEAR(wrap_pi(_ekf_wrapper.getYawAngle() - starting_yaw - physical_yaw_change),
				    0.f, radians(0.05f));
		}
	}

	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), reset_count);
	EXPECT_FALSE(_ekf->control_status_flags().mag_hdg);
	EXPECT_FALSE(_ekf->control_status_flags().mag_3D);
}

INSTANTIATE_TEST_SUITE_P(EarthRate,
			 EkfEarthRateGyroscopeTest,
			 ::testing::Values(
				 std::make_tuple(40.41359f, 0.f, 0.f, 0.f),
				 std::make_tuple(40.41359f, 0.f, 0.f, 90.f),
				 std::make_tuple(40.41359f, 0.f, 0.f, 179.f),
				 std::make_tuple(40.41359f, 0.f, 0.f, -90.f),
				 std::make_tuple(-40.41359f, 0.f, 0.f, 45.f),
				 std::make_tuple(80.f, 0.f, 0.f, -179.f),
				 std::make_tuple(40.41359f, 12.f, -20.f, 156.f),
				 std::make_tuple(-40.41359f, -15.f, 18.f, -130.f)
			 ));
