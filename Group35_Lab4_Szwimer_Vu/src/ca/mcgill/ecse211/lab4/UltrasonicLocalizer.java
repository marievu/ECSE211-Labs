package ca.mcgill.ecse211.lab4;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.*;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class is the ultrasonic localizer. We first place the robot along the 45 degree line of the bottom left
 * corner of the 4x4 grid field. With the method fallingEdge or risingEdge, the robot will approximately localize
 * the 0 degree orientation using the ultrasonic sensor.
 */
public class UltrasonicLocalizer extends Thread implements UltrasonicController {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Odometer odometer;
	
	private static int ROTATE_SPEED = Lab4.ROTATE_SPEED;
	private static double WHEEL_RAD = Lab4.WHEEL_RAD;
	private static double TRACK = Lab4.TRACK;
	
	private SampleProvider us;
	private float[] usData;
	public boolean isEdge;
	private final int FILTER_OUT = 20;
	
	private int distance;
	double angle;
	boolean firstDetection = true;
	boolean secondDetection = true;
	int cntr;      // used to count the number of detections
	
	/**
	 * Constructor
	 * @param leftMotor
	 * @param rightMotor
	 * @param odometer
	 * @param us
	 * @param usData
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer,
			SampleProvider us, float[] usData) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;
		this.us = us;
		this.usData = usData;
		leftMotor.setAcceleration(600);     //makes the robot smoother
		rightMotor.setAcceleration(600);
	}
	
	/**
	 * Runs the ultrasonic localizer. 
	 * Based on the button that is pressed (falling edge or rising edge) a method is chosen to run.
	 */
	public void run() {

		while (true) {
			us.fetchSample(usData, 0);
			distance = (int) (usData[0] * 100.0);
			
			LCD.drawString(Integer.toString(distance), 0, 6);
	
			//rising edge
			if (Lab4.isEdge == true) {
			
				risingEdge(distance);
			} 
			//falling edge
			else { 
				fallingEdge(distance);
			}
			if (secondDetection == false) {
				odometer.setTheta(0);
				break;
			}
		}
	}

	
	/**
	 * The fallingEdge method takes distance read by the ultrasonic sensor to determine the angle that we need to turn by
	 * to face the 0 degree direction. The robot turns left towards the wall and detect the first falling edge at 45cm.
	 * It then turn to the right, facing away from the wall, until it detects the second falling edge.  It then calculates
	 * the theta needed to adjust the robot to the right axis.
	 * 
	 * @param distance
	 */
	public void fallingEdge(int distance) {

		Lab4.leftMotor.setSpeed(ROTATE_SPEED);
		Lab4.rightMotor.setSpeed(ROTATE_SPEED);

		//begin turning left towards the wall (while facing away from the walls) to detect first edge
		if (firstDetection) {
			Lab4.leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 360), true); //turns left until first edge found
			Lab4.rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 360), true);

			if (distance < 45) {        //first edge located, increment counter to 1 and firstDetection to false
				cntr = 1; 
				firstDetection = false; 
			}
			secondDetection = true;

		}
		
		// if the first edge has been detected and the counter is equal to 1, reset the angle on the odometer to 0 to measure the angle
		// between the first and second edge. 
		else if (distance < 45 && cntr == 1) {
			
			leftMotor.stop(true);
			rightMotor.stop(false);
			odometer.setTheta(0);
			Sound.beep();
			//turning the robot to the right (towards second edge) by 80 degrees to avoid the sensor from reading unwanted values
			Lab4.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 80), true);
			Lab4.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 80), false); 
			Sound.beep();
			cntr = 2;                  //set counter to 2, since this will be the second detection
			secondDetection = true;
		
		
		}
		//search for the next falling edge by rotating facing away from the wall
		else if (distance > 45 && cntr == 2) { 
			
			Lab4.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 360), true);
			Lab4.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 360), true); // right turn
			secondDetection = true;
		
		} 
		
		//second edge is located
		else if (distance < 45 && cntr == 2) { 
			
			leftMotor.stop(true);
			rightMotor.stop(false);
			angle = odometer.getXYT()[2]; //acquire theta between two edges
			Sound.beep();
			turn(false, 0.5 * angle); //adjust robots orientation to be 0 degrees, along the axis based on the angle between the 2 edges
			turn(false, 37);          //turn the robot to face the 0 axis.
			leftMotor.stop(true);
			rightMotor.stop(false);
			cntr = 3;
			secondDetection = false;

		}

	}
	
	
	/**
	 *The risingEdge method takes distance read by the ultrasonic sensor to determine the angle that we need to turn by
	 *to face the 0 degree direction. The robot is first facing a wall and it looks for a rising edge which is defined
	 *to be at 45cm. Once it has been detected, our robot turns the opposite way to find the second rising edge.
	 *It then calculates the theta between the 2 edges and localizes robots' angle.
	 *
	 *@param distance  
	 */
	public void risingEdge(int distance) {

		Lab4.leftMotor.setSpeed(ROTATE_SPEED); 
		Lab4.rightMotor.setSpeed(ROTATE_SPEED);
		
		//rotate the robot to the left until it detects the first rising edge at 45cm. 
		if (firstDetection) { 
			Lab4.leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 360), true); 
			Lab4.rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 360), true); 

			if (distance > 45) {       //rising edge detected, increment the counter of detection and set the first detection to false
				cntr = 1;            
				firstDetection = false;
			}
			secondDetection = true; 

		}
		
		// if the first edge has been detected and the counter is equal to 1, reset the angle on the odometer to 0 to measure the angle
		// between the first and second edge. 
		else if (distance > 45 && cntr == 1) {
			
			leftMotor.stop(true);
			rightMotor.stop(false);
			odometer.setTheta(0);
			Sound.beep();
			//turning the robot to the right (towards second edge) by 65 degrees to avoid the sensor from reading unwanted values
			Lab4.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 65), true); 
			Lab4.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 65), false);
			Sound.beep();
			cntr = 2;        //set counter to 2, since this will be the second detection
			secondDetection = true;
			
		//search for the next rising edge
		} else if (distance < 45 && cntr == 2) {
			
			//turn right until the second edge is detected
			Lab4.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 360), true); 
			Lab4.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 360), true);
			secondDetection = true;
			
		
		} 
		//second edge is detected 
		else if (distance > 45 && cntr == 2) {
			
			leftMotor.stop(true);
			rightMotor.stop(false);
			angle = odometer.getXYT()[2];
			Sound.beep();
			turn(false, 0.5 * angle); // facing 45 degrees
			turn(false, 221);         // facing forward along the 0 axis
			leftMotor.stop(true); 
			rightMotor.stop(false);
			cntr = 3;
			secondDetection = false;		
		}
	}
	
	/**
	 * This method was found it the square driver class of previous labs. It is used to rotate the robot 
	 * in place based on the angle we want.
	 * 
	 * @param direction
	 * @param angle
	 */
	public static void turn(boolean direction, double angle) {
		Lab4.leftMotor.setSpeed(ROTATE_SPEED);
		Lab4.rightMotor.setSpeed(ROTATE_SPEED);

		if (direction == true) {
			Lab4.leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), true);
			Lab4.rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), false);
		} else {
			Lab4.leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, angle), true);
			Lab4.rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, angle), false);
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
	 *  total rotation of each wheel need to cover that distance.
	 * @param WHEEL_RAD
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double WHEEL_RAD, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * WHEEL_RAD));
	}
	

	/**
	 * filter and process for the ultrasonic sensor
	 */
	@Override
	public void processUSData(int distance) {

		int filterControl = 0;
		if (distance >= 255 && filterControl < FILTER_OUT) {
			filterControl++; 
		} else if (distance >= 255) {
			this.distance = distance;
		} else {
			filterControl = 0;
			this.distance = distance;
		}
	}
	
	@Override
	public int readUSDistance() {
		return this.distance;
	}
}