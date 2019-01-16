package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

/**
 *This class is where all the motors and sensors are instantiated. It is where the
 * menu screen on the robot is created that allows to run the UltrasonicLocalizer.
 */


public class Lab4 extends Thread {
	
	
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); 
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	public static final EV3ColorSensor colorPort = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	
	public static final double WHEEL_RAD = 2.05;
	public static final double TRACK = 14.7;
	public static final int FORWARD_SPEED = 120;
	public static final int ROTATE_SPEED = 60;
	public static final double bandcenter = 15;
	
	
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static boolean isEdge;

	public static void main(String[] args) throws Exception {
		int btnchoice;

		//setup ultrasonic sensor
		@SuppressWarnings("resource")
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usDistance = usSensor.getMode("Distance");
		float[] usData = new float[usDistance.sampleSize()];

		Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Display odometryDisplay = new Display(lcd);

		UltrasonicLocalizer UltrasonicLoc = new UltrasonicLocalizer(leftMotor, rightMotor, odometer, usDistance, usData);

		LightLocalizer LightLocThread = new LightLocalizer(leftMotor, rightMotor, odometer, colorPort);

		do {
			lcd.clear();

			lcd.drawString(" < Left | Right > ", 0, 0);
			lcd.drawString("        |         ", 0, 1);
			lcd.drawString("Falling | Rising  ", 0, 2);
			lcd.drawString(" Edge   |  Edge   ", 0, 3);
			
		btnchoice = Button.waitForAnyPress();
		} while (btnchoice != Button.ID_LEFT && btnchoice != Button.ID_RIGHT);

		Thread odoThread = new Thread(odometer);
		Thread odoDisplayThread = new Thread(odometryDisplay);
		UltrasonicPoller usPoller = null;

		Thread LightLoc = new Thread(LightLocThread);

		//Falling edge
		if (btnchoice == Button.ID_LEFT) {
			lcd.clear();

			isEdge = false;
			usPoller = new UltrasonicPoller(usDistance, usData, UltrasonicLoc);

			odoThread.start();
			odoDisplayThread.start();
			UltrasonicLoc.start();

			
			//After UltrasonicLocalizer is done, press center button to activate LighLocalizer
			while (Button.waitForAnyPress() != Button.ID_ENTER)
				;
			UltrasonicLoc.join();	       //end Ultrasonic localizer
			LightLoc.start();              //begin LightLocalizer class

		//rising edge
		} else {
			lcd.clear();

			isEdge = true;
			usPoller = new UltrasonicPoller(usDistance, usData, UltrasonicLoc);

			odoThread.start();
			odoDisplayThread.start();
			UltrasonicLoc.start();

			while (Button.waitForAnyPress() != Button.ID_ENTER)
				; 
			
			UltrasonicLoc.join();                            // end Ultrasonic localizer
			System.out.println(UltrasonicLoc.isAlive());     //test if the thread is still alive
			LightLoc.start();                                //begin the LightLocalizer class

		}

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}
