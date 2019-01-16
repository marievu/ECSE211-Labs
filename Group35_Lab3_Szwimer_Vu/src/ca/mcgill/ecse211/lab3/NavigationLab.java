// / Lab2.java
package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class NavigationLab {

	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.05;
	public static final double TRACK = 14.7;
	public static final double radius = 2.05;
	public static final double track = 14.7;
	public static final int bandCenter = 30;
	public static final int bandWidth = 2;

	
	
	
	public static void main(String[] args) throws OdometerExceptions {
		
	    
		// Odometer related objects
		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		NavigationObstacle navigationObstacle = new NavigationObstacle(leftMotor, rightMotor, TRACK, WHEEL_RAD);

		
		Display odometryDisplay = new Display(lcd);

		// clear the display
		lcd.clear();

		 int buttonChoice;
		do {
		      // clear the display
		      lcd.clear();
		
		      // ask the user whether odometery correction should be run or not
		      lcd.drawString("< Left | Right >", 0, 0);
		      lcd.drawString("                ", 0, 1);
		      lcd.drawString("  Navi-|  Nav   ", 0, 2);
		      lcd.drawString(" gation|  Obst  ", 0, 3);
		  
		
		      buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		  	}	while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);
		  
				  Thread odoThread = new Thread(odometer);
			      odoThread.start();
			      Thread odoDisplayThread = new Thread(odometryDisplay);
			      odoDisplayThread.start();
			     
			      
			   //Navigation with obstacle
			  if (buttonChoice == Button.ID_RIGHT) {
				  
				  Thread navigationNoThread = new Thread(navigationObstacle);
				  navigationNoThread.start();
				} 
			  
		// spawn a new Thread to avoid Navigation from blocking
		//this tread is only for Navigation without obstacle
		(new Thread() {
			
			// Wait for any button press to start
			int buttonChoice = Button.waitForAnyPress();

			public void run() {
			// Map waypoints
				int waypoints1[][] = new int[][]  { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
				int waypoints2[][] = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
				int waypoints3[][] = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
				int waypoints4[][] = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };

				// Define navigation object with motor specifications
				Navigation nav = new Navigation(leftMotor, rightMotor, WHEEL_RAD, TRACK);

				// Loop to go to each waypoint
				for (int i = 0; i < 5; i++) {
					nav.travelTo(waypoints2[i][0], waypoints2[i][1]);
				}
			}
		}).start();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}
}
