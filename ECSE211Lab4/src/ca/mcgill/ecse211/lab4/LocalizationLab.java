package ca.mcgill.ecse211.lab4;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class LocalizationLab {

	private static final EV3LargeRegulatedMotor leftMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		  
		  private static final EV3LargeRegulatedMotor rightMotor =
		      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

		  public static final double RADIUS = 2.12; // radius of the wheel
		  public static final double TRACK = 11.7; 		  // distance between wheels

		  public static void main(String[] args) {
		    int buttonChoice;

		    final TextLCD t = LocalEV3.get().getTextLCD();
		    Odometer odometer = new Odometer(leftMotor, rightMotor);

		    do {
		      // clear the display
		      t.clear();

		      // ask the user whether the motors should drive in a square or float
		      t.drawString("< Left | Right >", 0, 0);
		      t.drawString("       |        ", 0, 1);
		      t.drawString(" Rising|Falling ", 0, 2);
		      t.drawString(" edge  | edge   ", 0, 3);
		      t.drawString("       |  		", 0, 4);

		      buttonChoice = Button.waitForAnyPress();
		    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		    if (buttonChoice == Button.ID_LEFT) {

		      

		    } else {
		      // clear the display
		      t.clear();

		      // ask the user whether the motors should drive in a square or float
		      t.drawString("< Left | Right >", 0, 0);
		      t.drawString("  No   | with   ", 0, 1);
		      t.drawString(" corr- | corr-  ", 0, 2);
		      t.drawString(" ection| ection ", 0, 3);
		      t.drawString("       |        ", 0, 4);
		      
		      buttonChoice = Button.waitForAnyPress();
		      
		      odometer.start();
		      
		      // spawn a new Thread to avoid SquareDriver.drive() from blocking
		    }

		    while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		    System.exit(0);
		  }
}
