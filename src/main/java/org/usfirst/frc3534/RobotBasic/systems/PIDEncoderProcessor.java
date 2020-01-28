package org.usfirst.frc3534.RobotBasic.systems;

public class PIDEncoderProcessor {
	
	double kp, kd, kv;
	
	double expectedPosition = 0, last_error = 0;
	
	double encoder_offset, encoder_tick_count, wheel_circumference;
	
	double dt;
	
	public PIDEncoderProcessor() {
		
	}
	
	/**
	 * Configure the PID/VA Variables for the Follower
	 * 
	 * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are
	 *           common values)
	 * @param ki The integral term. Currently unused.
	 * @param kd The derivative term. Adjust this if you are unhappy with the
	 *           tracking of the follower. 0.0 is the default
	 * @param kv The velocity ratio. This should be 1 over your maximum velocity @
	 *           100% throttle. This converts m/s given by the algorithm to a scale
	 *           of -1..1 to be used by your motor controllers
	 * @param ka The acceleration term. Adjust this if you want to reach higher or
	 *           lower speeds faster. 0.0 is the default
	 */
	public void configurePIDVA(double kp, double kd, double kv) {
		this.kp = kp;
		this.kd = kd;
		this.kv = kv;
	}
	
	/**
	 * Configure the Encoders being used in the follower.
	 * 
	 * @param initial_position     The initial 'offset' of your encoder. This should
	 *                             be set to the encoder value just before you start
	 *                             to track
	 * @param ticks_per_revolution How many ticks per revolution the encoder has
	 * @param wheel_diameter       The diameter of your wheels (or pulleys for track
	 *                             systems) in meters
	 */
	public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter, double deltaTime) {
		encoder_offset = initial_position;
		encoder_tick_count = ticks_per_revolution;
		wheel_circumference = Math.PI * wheel_diameter;
		dt = deltaTime;
		reset();
	}
	
	/**
	 * Calculate the desired output for the motors, based on the amount of ticks the
	 * encoder has gone through. This does not account for heading of the robot. To
	 * account for heading, add some extra terms in your control loop for
	 * realignment based on gyroscope input and the desired heading given by this
	 * object.
	 * 
	 * @param encoder_tick The amount of ticks the encoder has currently measured.
	 * @return The desired output for your motor controller
	 */
	public double calculate(int encoder_tick, double velocity) {
		// Number of Revolutions * Wheel Circumference
		double distance_covered = ((double) (encoder_tick - encoder_offset) / encoder_tick_count) * wheel_circumference;
		double error = expectedPosition - distance_covered;
		double calculated_value = kp * error + // Proportional
				kd * ((error - last_error) / dt) + // Derivative
				kv * velocity; // V Term
		last_error = error;
		expectedPosition += velocity * dt;

		return calculated_value;
	}

	private void reset() {
		
		expectedPosition = 0;
		last_error = 0;
		
	}
	
}
