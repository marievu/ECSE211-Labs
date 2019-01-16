package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;


public class Navigation implements Runnable {

	private static final int FORWARD_SPEED = 220;
	private static final int ROTATE_SPEED = 120;
	private static final double TILE_SIZE = 30.48;
	int iterator = 0;

	public static EV3LargeRegulatedMotor leftMotor;
	public static EV3LargeRegulatedMotor rightMotor;
	public static double WHEEL_RAD;
	public static double TRACK;
	


	public Navigation(EV3LargeRegulatedMotor leftmotor, EV3LargeRegulatedMotor rightmotor, 
			double WHEEL_RAD, double TRACK) {
		Navigation.leftMotor = leftmotor;
		Navigation.rightMotor = rightmotor;
		Navigation.WHEEL_RAD = WHEEL_RAD;
		Navigation.TRACK = TRACK;

	}
	
	// run method (required for Thread)
			public void run() {
				
			}

	/**
	 * This method causes the robot to travel to the absolute field location (x, y), specified in tile points.
	 *  This method should continuously call turnTo(double theta) and then set the motor speed to forward(straight). 
	 *  This will make sure that your heading is updated until you reach your exact goal. This method will poll the odometer for information.
	 * 
	 * @param x
	 * @param y
	 */
	private static double currAngle = 0;

	public void travelTo(double x, double y) {
		// Define variables
		double minAngle = 0, hypot = 0, deltaX = 0, deltaY = 0, odometer[] = {0,0,0};
		

		try {
			odometer = Odometer.getOdometer().getXYT();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
		
		navigating = true;

		x = x * TILE_SIZE;
		y = y * TILE_SIZE;

		currAngle = odometer[2];
  
		// difference between x and what the odometer displays
		deltaX = x - odometer[0];
		deltaY = y - odometer[1];

		// Displacement to waypoint (hypothenuse)
		hypot = Math.hypot(Math.abs(deltaX), Math.abs(deltaY));

		// Get absolute angle the robot must be facing
		minAngle = Math.toDegrees(Math.atan2(deltaX, deltaY));

		// if the angle is negative, make it positive
		if (minAngle < 0)
			minAngle = 360 - Math.abs(minAngle);

		// robot will turn to the minimal angle
		turnTo(minAngle);

		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);

		// go to the next waypoint
		leftMotor.rotate(convertDistance(WHEEL_RAD, hypot), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, hypot), false);
	}
	
	
	/**
	 * This method causes the robot to turn (on point) to the absolute heading theta. This method should turn a MINIMAL angle to its target.
	 * 
	 * @param theta
	 */
	
	public static void turnTo(double theta) {
		boolean turnLeft = false;
		double deltaAngle = 0;
		// Get change in angle we want
		deltaAngle = theta - currAngle;

		// If deltaAngle is negative, loop it back
		if (deltaAngle < 0) {
			deltaAngle = 360 - Math.abs(deltaAngle);
		}

		// Check if we want to move left or right
		if (deltaAngle > 180) {
			turnLeft = true;
			deltaAngle = 360 - Math.abs(deltaAngle);
		} else {
			turnLeft = false;
		}

		// Set slower rotate speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);

		// Turn motors according to which direction we want to turn in
		if (turnLeft) {
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaAngle), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaAngle), false);
		} else {
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, deltaAngle), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, deltaAngle), false);
		}

	}

	/**
	 *This method returns true if another thread has called travelTo() or turnTo() and the method has yet to return;
	 * false otherwise.
	 * 
	 */
	static boolean navigating = false;

	public static boolean isNavigating() {
		return navigating;
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param WHEEL_RAD
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double WHEEL_RAD, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}

	private static int convertAngle(double WHEEL_RAD, double width, double angle) {
		return convertDistance(WHEEL_RAD, Math.PI * width * angle / 360.0);
	}

}
