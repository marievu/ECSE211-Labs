package ca.mcgill.ecse211.lab3;
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerData;
import ca.mcgill.ecse211.odometer.OdometerExceptions;


public class NavigationObstacle implements Runnable {

	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private float[] usData;
	private SampleProvider usDistance ;
	
	private static Odometer odometer;
	private OdometerData odoData;
	
	private static double TRACK = 14.7;
	private static double WHEEL_RAD = 2.05;
	public static final int FORWARD_SPEED = 250;
	private static final int ROTATE_SPEED = 150;
	
	static double theta = 0;
	double currY;
	double currX;
	double deltaX, deltaY, deltaT;
	double travelDist;
	int counter = 0;
	
	
	private double[][]  wayPoints = new double[][]{
		{ 1*30.48, 1*30.48 }, { 0*30.48, 2*30.48 }, { 2*30.48, 2*30.48 }, { 2*30.48, 1*30.48 }, { 1*30.48, 0*30.48 }
	};
	
	/* waypoints:
	 * map 1: { 0*30.48, 2*30.48} , { 1*30.48, 1*30.48 }, { 2*30.48, 2*30.48 }, { 2*30.48, 1*30.48 }, { 1*30.48, 0*30.48 }
	 * map 2: { 1*30.48, 1*30.48 }, { 0*30.48, 2*30.48 }, { 2*30.48, 2*30.48 }, { 2*30.48, 1*30.48 }, { 1*30.48, 0*30.48 }
	 * map 3: { 1*30.48, 0*30.48 }, { 2*30.48, 1*30.48 }, { 2*30.48, 2*30.48 }, { 0*30.48, 2*30.48 }, { 1*30.48, 1*30.48 }
	 * map 4: { 0*30.48, 1*30.48 }, { 1*30.48, 2*30.48 }, { 1*30.48, 0*30.48 }, { 2*30.48, 1*30.48 }, { 2*30.48, 2*30.48 }
	 * */
      
    //constructor
	public NavigationObstacle(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
		NavigationObstacle.odometer = Odometer.getOdometer();
		NavigationObstacle.leftMotor = leftMotor;
		NavigationObstacle.rightMotor = rightMotor;
		odoData = OdometerData.getOdometerData();
		odoData.setXYT(0 , 0 , 0);
		NavigationObstacle.TRACK = TRACK;
		NavigationObstacle.WHEEL_RAD = WHEEL_RAD;
		//setup our ultrasonic sensor
		SensorModes usSensor = new EV3UltrasonicSensor(usPort); 
		usDistance = usSensor.getMode("Distance"); 
		this.usData = new float[usDistance.sampleSize()]; 		
			
		}

		// run method (required for Thread)
	public void run() {
		//this make the robot run smoother
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(250);  
		}
		try {
			Thread.sleep(2000);
		} catch (InterruptedException e) {
		}
		
		
		// while loop iterates through the waypoints
		while(counter < wayPoints.length) {
			travelTo(wayPoints[counter][0], wayPoints[counter][1]);
			counter++;
		}
	}
	

	/**
	 * This method causes the robot to travel to the absolute field location (x, y), specified in tile points.
	 * This method should continuously call turnTo(double theta) and then set the motor speed to forward(straight). 
	 * This will make sure that your heading is updated until you reach your exact goal. This method will poll the odometer for information.
	 * This method also avoids if there is an obstacle on the way by going around it in a square like manner.
	 * 
	 * @param x
	 * @param y
	 */
	
	void travelTo(double x, double y) {
		currX = odometer.getXYT()[0];// get the position on the board
		currY = odometer.getXYT()[1];
		theta = odometer.getXYT()[2];

		deltaX = x- currX;
		deltaY = y - currY;
		travelDist = Math.sqrt(deltaX*deltaX+deltaY*deltaY);
		if(deltaY>=0) {
			deltaT=Math.atan(deltaX/deltaY);
		}
		else if(deltaY<=0&&deltaX>=0) {
			deltaT=Math.atan(deltaX/deltaY)+Math.PI;
		}
		else {
			deltaT=Math.atan(deltaX/deltaY)-Math.PI;
		}

		// initial angle is  0 same direction as y-axis, going clockwise
		double differenceInTheta = (deltaT*180/Math.PI-theta); // robot has to turn "differenceInTheta",
		//turn the robot to the desired direction
		turnTo(differenceInTheta); 

		// drive forward by the distance needed
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.rotate(convertDistance(WHEEL_RAD, travelDist), true);
		rightMotor.rotate(convertDistance(WHEEL_RAD, travelDist), true);

		//this while loop avoid the obstacle if there is one
		while(isNavigating()) {
			usDistance.fetchSample(usData,0);
			float distance = usData[0]*100;
			
			//if distance fetched is smaller than 13, that means its too close to the obstacle, turn 90 degrees and avoid the block
			if(distance<= 13) {
				
				//turn to the left if we are running on map 3 and travel a distance
				if(odometer.getXYT()[0] < 2.5*30.48 && odometer.getXYT()[0] > 1.5*30.48 
						&& odometer.getXYT()[1] < 2.5*30.48&&odometer.getXYT()[1] > 1.5*30.48){
					leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);  
					rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
					leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);
					rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
					leftMotor.rotate(convertDistance(WHEEL_RAD, 40), true);
					rightMotor.rotate(convertDistance(WHEEL_RAD, 40), false);
				}
				//else turn right and travel a distance
				else {
				leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), true);   
				rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), false);
				leftMotor.rotate(convertDistance(WHEEL_RAD, 30), true);
				rightMotor.rotate(convertDistance(WHEEL_RAD, 30), false);
				leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, 90), true);
				rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, 90), false);
				leftMotor.rotate(convertDistance(WHEEL_RAD, 35), true);
				rightMotor.rotate(convertDistance(WHEEL_RAD, 35), false);
				}
				counter--;
			}
		}
	}
		
	/**
	 * This method causes the robot to turn (on point) to the absolute heading theta. This method should turn a MINIMAL angle to its target.
	 * 
	 * @param theta
	 */
	void turnTo(double theta) {
		
		if(theta>180) {
			theta=360-theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
		//
		else if(theta<-180) {
			theta=360+theta;
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);
		}
	
		else {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(convertAngle(WHEEL_RAD, TRACK, theta), true);
			rightMotor.rotate(-convertAngle(WHEEL_RAD, TRACK, theta), false);	
		}
	}

	/**
	 *This method returns true if another thread has called travelTo() or turnTo() and the method has yet to return;
	 * false otherwise.
	 * 
	 */
		
	boolean isNavigating() {
		if((leftMotor.isMoving() || rightMotor.isMoving()))
			return true;
		else 
			return false;
		}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param WHEEL_RAD
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	private static int convertAngle(double radius, double wideltaTh, double angle) {
		return convertDistance(radius, Math.PI * wideltaTh * angle / 360.0);
	}
}