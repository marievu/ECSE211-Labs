package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * Robot uses the light sensor to detect black lines. It then moves to the (0,0) point designated on the 4x4 grid field.
 */
public class LightLocalizer extends OdometerData implements Runnable {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	
	private Odometer odometer;
	private EV3ColorSensor colorSensor;

	
	private int color;
	public double data;
	private static int FORWARD_SPEED = Lab4.FORWARD_SPEED;
	private static double WHEEL_RAD = Lab4.WHEEL_RAD;
	private static double track = Lab4.TRACK;
	
	private boolean firstLine = true;
	private boolean secondLine = true;
	private boolean blackLineDetected = false;
	int cntr;

	/**
	 * Constructor
	 * @param leftMotor
	 * @param rightMotor
	 * @param odometer
	 * @param colorSensor
	 * @throws Exception
	 */
	public LightLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			EV3ColorSensor colorSensor) throws Exception {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.colorSensor = colorSensor;
		leftMotor.setAcceleration(600);
		rightMotor.setAcceleration(600);

	}

	/**
	 * Runs the light localizer.
	 * sets the offset of our light sensor at 11 which is measured from the wheels of the robot.
	 */
	public void run() {
		Lab4.leftMotor.setSpeed(FORWARD_SPEED);
		Lab4.rightMotor.setSpeed(FORWARD_SPEED);

		odometer.setY(11);
		odometer.setTheta(0);
		while (true) {
			color = colorSensor.getColorID();
			localization(color);

			//exit this loop
			if (secondLine == false) { 
				break;
			}
		}
	}
	
	
	/**
	 * The localization method is used to detect the black lines on the grid field in order to go to the (0,0) point.
	 * The light sensor detects a black line, stops and goes back to the line by its length (17cm). It then turns 90 degrees
	 * to the right and does the same thing for the second line.
	 * @param line
	 */
	public void localization(int line) {
		
		// Goes straight until there's a line to detect
		if (firstLine == true && lineDetected(line) == false) {
			Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 50), true);
			Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 50), true);
			secondLine = true;
			
		
		//first line is found
		} else if (firstLine == true && lineDetected(line) == true) {
			leftMotor.stop(true);
			rightMotor.stop(false);
			try {
				Thread.sleep(1000);
			} catch (Exception e) {
			}
			
			moveRobot(false, 17);         //move back 17cm because thats the light sensor's offset
			odometer.setY(0);
			turn(true, 90);               //turn 90 degrees along the x axis
			cntr = 1;
			secondLine = true;
			firstLine = false;
			
		
		}
		//goes straight until the second line is detected
		else if (cntr == 1 && lineDetected(line) == false) {

			Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 50), true);
			Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, 50), true);
			secondLine = true;
			
		
		}
		//detected second black line 
		else if (cntr == 1 && lineDetected(line) == true) { 
			leftMotor.stop(true);
			rightMotor.stop(false);
			try {
				Thread.sleep(1000);
			} catch (Exception e) {
			}
			
			moveRobot(false, 17);          //move back 17cm because thats the light sensor's offset
			odometer.setX(0);
			turn(false, 90);               //turn 90 degrees again 
			cntr = 2; 
			odometer.setXYT(0, 0, 0);
			secondLine = false; 
		}
	}

	/**
	 * This method checks if the line detected is black and returns true if it is.
	 * 
	 * @param line
	 * @return
	 */
	public boolean lineDetected(int line) {
		while (true) {
			//determines based on colour ID if line was detected
			if (line > 10) {
				blackLineDetected = true; //if so return true
				Sound.beepSequenceUp();

			} else {
				blackLineDetected = false; // no line detected returns false
			}
			return blackLineDetected;
		}

	}
	
	/**
	 * This method was found it the square driver class of previous labs. It is used to rotate the robot 
	 * in place based on the angle we want.
	 * 
	 * @param direction
	 * @param angle
	 */
	public void turn(boolean direction, double angle) {

		if (direction == true) {
			Lab4.leftMotor.rotate(convertAngle(WHEEL_RAD, track, angle), true);
			Lab4.rightMotor.rotate(-convertAngle(WHEEL_RAD, track, angle), false);
		} else {
			Lab4.leftMotor.rotate(-convertAngle(WHEEL_RAD, track, angle), true);
			Lab4.rightMotor.rotate(convertAngle(WHEEL_RAD, track, angle), false);
		}
		leftMotor.stop(true);
		rightMotor.stop(false);
	}
	
	/**
	 * a method from previous lab to move robot based on input distance
	 * @param direction
	 * @param distance
	 */
	public void moveRobot(boolean direction, double distance) {
		Lab4.leftMotor.setSpeed(FORWARD_SPEED);
		Lab4.rightMotor.setSpeed(FORWARD_SPEED);

		if (direction == true) {
			Lab4.leftMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), true);
			Lab4.rightMotor.rotate(convertDistance(Lab4.WHEEL_RAD, distance), false);
		}

		else {
			Lab4.leftMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, distance), true);
			Lab4.rightMotor.rotate(-convertDistance(Lab4.WHEEL_RAD, distance), false);
		}
	}
	
	/**
	 * This method was taken from previous labs to convert angle between rads and degrees.
	 * @param WHEEL_RAD
	 * @param width
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double WHEEL_RAD, double width, double angle) {
		return convertDistance(WHEEL_RAD, Math.PI * width * angle / 360.0);
	}
	/**
	 * This method was taken from previous labs to allow the conversion of a distance to the
	 * total rotation of each wheel need to cover that distance.
	 * @param WHEEL_RAD
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double WHEEL_RAD, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}
}