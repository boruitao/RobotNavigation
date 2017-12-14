package ca.mcgill.ecse211.project;

import java.util.Arrays;
/** Main class
 * 	@author Borui Tao
 * 	@version 1.0
 * 
 */
import java.util.Map;
import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

/**
 * This class is the main one and interacts with the other classes to perform the localization, navigation
 * and zipline traversal.
 */
public class CaptureFlag {


	//Set these as appropriate for your team and current situation
	/**
	 * The server IP for the computer "controlling" the robot.
	 */
	private static final String SERVER_IP = "192.168.2.10";

	/**
	 * The team number.
	 */
	private static final int TEAM_NUMBER = 8;

	//Enable/disable printing of debug info from the WiFi class
	private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

	/**
	 * The pointer to the left motor.
	 */
	private static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

	/**
	 * The pointer to the right motor.
	 */
	private static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	/**
	 * The pointer to the zipline motor.
	 */
	private static final EV3LargeRegulatedMotor ziplineMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));

	/**
	 * The pointer to the motor connected to the ultrasonic sensor (to turn it 45 degrees when detecting a flag).
	 */
	private static final EV3LargeRegulatedMotor ultrasonicMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));

	/**
	 * The pointer to the ultrasonic sensor.
	 */
	private static final Port usPort = LocalEV3.get().getPort("S1");

	/**
	 * The ultrasonic localizer.
	 */
	private static UltrasonicLocalizer ultraLoc;
	//	private static BangBangController bb;

	/**
	 * The light localizer.
	 */
	private static LightLocalizer lightLoc;
	/**
	 * The value got by the front light sensor an updated by the LightSensor class.
	 */
	public static float lightValue;
	/**
	 * The speed at which the robot is traveling straight.
	 */
	public static final int FORWARD_SPEED = 270; //140
	/**
	 * The distance between the Ultrasonic sensor and the back light sensor.
	 */
	public static final double ROBOT_LENGTH = 10.2;
	/**
	 * The speed at which the robot is rotating.
	 */
	public static final int ROTATE_SPEED = 150; // 110
	/**
	 * The radius of the wheel.
	 */
	public static final double RADIUS = 2.12;
	/**
	 * The distance between the wheels.
	 */
	public static final double TRACK = 11.3;
	/**
	 * The distance from which the ultrasonic sensor detects the wall.
	 */
	public static final int DISTANCE_THRESHOLD = 50;
	/**
	 * The noise margin.
	 */
	public static final int NOISE_MARGIN = 1;
	/**
	 * The length of the square tile.
	 */
	public static final double SQUARE_LENGTH = 30.48;
	/**
	 * The speed at which the robot is traveling when mounting the zipline.
	 */
	public static final int ZIPLINE_SPEED = 300;
	/**
	 * The length of the zipline.
	 */
	public static final double ZIPLENGTH =70;
	/**
	 * The coordinates for the different corners.
	 */
	public static final int[][] CORNERS = new int[][] {
		{1,1,0},
		{5,1,270},
		{11,11,180},
		{1,11,90}
	};

	/**
	 * Using the helper classes, the main method takes care of the localization, navigation and river traversal.
	 * 
	 */
	//@SuppressWarnings("rawtypes")
	public static void main(String[] args) {

		int redTeam = 0, greenTeam = 0;
		int redCorner = 0, greenCorner = 0;
		int og = 0, or = 0;
		int red_ll_x = 0, red_ll_y = 0;
		int red_ur_x = 0, red_ur_y = 0;
		int green_ll_x = 0, green_ll_y = 0;
		int green_ur_x = 0, green_ur_y = 0;
		int zc_r_x = 0, zc_r_y = 0;
		int zo_r_x = 0, zo_r_y = 0;
		int zc_g_x = 0, zc_g_y = 0;
		int zo_g_x = 0, zo_g_y = 0;
		int sh_ll_x = 0, sh_ll_y = 0;
		int sh_ur_x = 0, sh_ur_y = 0;
		int sv_ll_x = 0, sv_ll_y = 0;
		int sv_ur_x = 0, sv_ur_y = 0;
		int sr_ll_x = 0, sr_ll_y = 0;
		int sr_ur_x = 0, sr_ur_y = 0;
		int sg_ll_x = 0, sg_ll_y = 0;
		int sg_ur_x = 0, sg_ur_y = 0;
		WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

		// Connect to server and get the data, catching any errors that might occur
		try {
			Map data = conn.getData();

			// Example 1: Print out all received data
			System.out.println("Map:\n" + data);

			redTeam = ((Long) data.get("RedTeam")).intValue();
			System.out.println("Red Team: " + redTeam);
			greenTeam = ((Long) data.get("GreenTeam")).intValue();
			System.out.println("Green Team: " + greenTeam);

			redCorner = ((Long) data.get("RedCorner")).intValue();
			System.out.println("Red Corner: " + redCorner);
			greenCorner = ((Long) data.get("GreenCorner")).intValue();
			System.out.println("Green Corner: " + greenCorner);

			og = ((Long) data.get("OG")).intValue();
			System.out.println("Green opponent flag: " + og);
			or = ((Long) data.get("OR")).intValue();
			System.out.println("Red opponent flag: " + or);

			red_ll_x =  ((Long) data.get("Red_LL_x")).intValue();
			System.out.println("Data: " + red_ll_x);
			red_ll_y =  ((Long) data.get("Red_LL_y")).intValue();
			System.out.println("Data: " + red_ll_y);
			red_ur_x =  ((Long) data.get("Red_UR_x")).intValue();
			System.out.println("Data: " + red_ur_x);
			red_ur_y =  ((Long) data.get("Red_UR_y")).intValue();
			System.out.println("Data: " + red_ur_y);
			green_ll_x =  ((Long) data.get("Green_LL_x")).intValue();
			System.out.println("Data: " + green_ll_x);
			green_ll_y =  ((Long) data.get("Green_LL_y")).intValue();
			System.out.println("Data: " + green_ll_y);
			green_ur_x =  ((Long) data.get("Green_UR_x")).intValue();
			System.out.println("Data: " + green_ur_x);
			green_ur_y =  ((Long) data.get("Green_UR_y")).intValue();
			System.out.println("Data: " + green_ur_y);
			zc_r_x =  ((Long) data.get("ZC_R_x")).intValue();
			System.out.println("ZC_R_x: " + zc_r_x);
			zc_r_y =  ((Long) data.get("ZC_R_y")).intValue();
			System.out.println("ZC_R_y: " + zc_r_y);
			zo_r_x =  ((Long) data.get("ZO_R_x")).intValue();
			System.out.println("ZO_R_x" + zo_r_x);
			zo_r_y =  ((Long) data.get("ZO_R_y")).intValue();
			System.out.println("ZO_R_y: " + zo_r_y);
			zc_g_x =  ((Long) data.get("ZC_G_x")).intValue();
			System.out.println("ZC_G_x: " + zc_g_x);
			zc_g_y =  ((Long) data.get("ZC_G_y")).intValue();
			System.out.println("ZC_G_y" + zc_g_y);
			zo_g_x =  ((Long) data.get("ZO_G_x")).intValue();
			System.out.println("ZO_G_x: " + zo_g_x);
			zo_g_y =  ((Long) data.get("ZO_G_y")).intValue();
			System.out.println("ZO_G_y: " + zo_g_y);
			sh_ll_x =  ((Long) data.get("SH_LL_x")).intValue();
			System.out.println("Data: " + sh_ll_x);
			sh_ll_y =  ((Long) data.get("SH_LL_y")).intValue();
			System.out.println("Data: " + sh_ll_y);
			sh_ur_x =  ((Long) data.get("SH_UR_x")).intValue();
			System.out.println("Data: " + sh_ur_x);
			sh_ur_y =  ((Long) data.get("SH_UR_y")).intValue();
			System.out.println("Data: " + sh_ur_y);
			sv_ll_x =  ((Long) data.get("SV_LL_x")).intValue();
			System.out.println("Data: " + sv_ll_x);
			sv_ll_y =  ((Long) data.get("SV_LL_y")).intValue();
			System.out.println("Data: " + sv_ll_y);
			sv_ur_x =  ((Long) data.get("SV_UR_x")).intValue();
			System.out.println("Data: " + sv_ur_x);
			sv_ur_y =  ((Long) data.get("SV_UR_y")).intValue();
			System.out.println("Data: " + sv_ur_y);
			sr_ll_x =  ((Long) data.get("SR_LL_x")).intValue();
			System.out.println("Data: " + sr_ll_x);
			sr_ll_y =  ((Long) data.get("SR_LL_y")).intValue();
			System.out.println("Data: " + sr_ll_y);
			sr_ur_x =  ((Long) data.get("SR_UR_x")).intValue();
			System.out.println("Data: " + sr_ur_x);
			sr_ur_y =  ((Long) data.get("SR_UR_y")).intValue();
			System.out.println("Data: " + sr_ur_y);
			sg_ll_x =  ((Long) data.get("SG_LL_x")).intValue();
			System.out.println("Data: " + sg_ll_x);
			sg_ll_y =  ((Long) data.get("SG_LL_y")).intValue();
			System.out.println("Data: " + sg_ll_y);
			sg_ur_x =  ((Long) data.get("SG_UR_x")).intValue();
			System.out.println("Data: " + sg_ur_x);
			sg_ur_y =  ((Long) data.get("SG_UR_y")).intValue();
			System.out.println("Data: " + sg_ur_y);
		} catch (Exception e) {
			System.err.println("Error: " + e.getMessage());
		}

		// clear the display
		//@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes ultrasonicSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = ultrasonicSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[1];		// usData is the buffer in which data are returned
		UltrasonicPoller usPoller = null;									// the selected controller on each cycle


		//two classes to perform wall following on the flags and detect the color of each one.
		LightSensor ls = new LightSensor();

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation na = new Navigation(odometer,leftMotor, rightMotor, ziplineMotor, ultrasonicMotor);
		//	bb = new BangBangController(na,odometer, 5,1,FORWARD_SPEED,ROTATE_SPEED,leftMotor,rightMotor);

		ultraLoc = new UltrasonicLocalizer(na, odometer, leftMotor, rightMotor);

		odometer.start();
		OdometryDisplay od = new OdometryDisplay(odometer, t,ultraLoc);
		od.start();
		//	usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc, bb);
		usPoller = new UltrasonicPoller(usDistance, usData, ultraLoc);
		lightLoc = new LightLocalizer(odometer, na);
		usPoller.start();

		//comment for testing purpose
		ultraLoc.doUltrasonicLocalization();
		//this method make the robot close to the actual (0,0) point
		lightLoc.adjustPosition();
		lightLoc.doLightLocalization();
		//Button.waitForAnyPress();

		if (TEAM_NUMBER == redTeam){
			na.makeTurn(2.7, false,false);
			odometer.setX(CORNERS[redCorner][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[redCorner][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[redCorner][2]);
			boolean gotToDestination = false;
			if(CORNERS[redCorner][0] != sr_ur_x && CORNERS[redCorner][1] != sr_ur_y){
				while(!gotToDestination){
					if (Math.abs(sr_ur_x-CORNERS[redCorner][0]) > Math.abs(sr_ur_y-CORNERS[redCorner][1]))
						gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, CORNERS[redCorner][1]*SQUARE_LENGTH);
					else 
						gotToDestination = na.travelTo(CORNERS[redCorner][0]*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5.8, false,false);
					System.out.println("lightLoc1");
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
				double cTheta = odometer.getTheta();
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5.0, false,true);
					while(na.isNavigating()){
					}
				}
				System.out.println("lightLoc2");
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}

			//SHALLOW CROSSING

			double centerRed_x = Math.abs(red_ur_x+red_ll_x)/2;
			double centerRed_y = Math.abs(red_ur_y+red_ll_y)/2;
			System.out.println("center " + centerRed_x + ";" + centerRed_y);
			double[] distances = new double[4];//0: sh_ll / 1: sh_ur / 2:sv_ll / 3:sv_ur
			int thirdPoint;
			gotToDestination = false;
			distances[0] = Math.sqrt(Math.pow(sh_ll_y-centerRed_y,2) + Math.pow(sh_ll_x-centerRed_x,2));
			distances[1] = Math.sqrt(Math.pow(sh_ur_y-centerRed_y,2) + Math.pow(sh_ur_x-centerRed_x,2));
			distances[2] = Math.sqrt(Math.pow(sv_ll_y-centerRed_y,2) + Math.pow(sv_ll_x-centerRed_x,2));
			distances[3] = Math.sqrt(Math.pow(sv_ur_y-centerRed_y,2) + Math.pow(sv_ur_x-centerRed_x,2));
			for(int k=0;k<4;k++){
				System.out.println("distance " + k + " - " + distances[k]);
			}
			int firstPoint = getMinDistance(distances);
			System.out.println(firstPoint);
			System.out.println("test print");
			if(firstPoint == 0 || firstPoint == 1){
				if(firstPoint == 0){
					System.out.println("going to sh_ll");
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					System.out.println("X first point: " + odometer.getX());
					System.out.println("Y first point: " + odometer.getY());
					System.out.println("theta first point: " + odometer.getTheta());
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					System.out.println("X second point: " + odometer.getX());
					System.out.println("Y second point: " + odometer.getY());
					System.out.println("theta second point: " + odometer.getTheta());
				}else if(firstPoint == 1){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
				}
				distances = new double[2];
				distances[0] = Math.sqrt(Math.pow(sv_ll_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sv_ll_x-(odometer.getX()/SQUARE_LENGTH),2));
				distances[1] = Math.sqrt(Math.pow(sv_ur_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sv_ur_x-(odometer.getX()/SQUARE_LENGTH),2));
				thirdPoint = getMinDistance(distances);

				//if the point where you go is behind you (in case the two rectangles are superimposed).
				/*int mid_x = (sh_ur_x + sh_ll_x)/2;
				int mid_y = (sh_ur_y + sh_ll_y)/2;
				if(((mid_x > sv_ll_x) && (mid_x < sv_ur_x)) && ((mid_y > sv_ll_y) && (mid_y < sv_ur_y))){
					thirdPoint = -1;
				}*/
				System.out.println("X point: " + odometer.getX());
				System.out.println("Y point: " + odometer.getY());
				System.out.println("theta point: " + odometer.getTheta());
				System.out.println("distance 0 " + distances[0]);
				System.out.println("distance 1 " + distances[1]);
				System.out.println(thirdPoint);
				if(thirdPoint == 0){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					na.travelTo((sv_ur_x-1)*SQUARE_LENGTH,(sv_ur_y+1)*SQUARE_LENGTH);
				}else if(thirdPoint == 1){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					na.travelTo((sv_ll_x+1)*SQUARE_LENGTH,(sv_ll_y-1)*SQUARE_LENGTH);
				}
			}

			if(firstPoint == 2 || firstPoint == 3){
				if(firstPoint == 2){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
				}else if(firstPoint == 3){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
				}
				distances = new double[2];
				distances[0] = Math.sqrt(Math.pow(sh_ll_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sh_ll_x-(odometer.getX()/SQUARE_LENGTH),2));
				distances[1] = Math.sqrt(Math.pow(sh_ur_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sh_ur_x-(odometer.getX()/SQUARE_LENGTH),2));
				thirdPoint = getMinDistance(distances);

				//if the point where you go is behind you (in case the two rectangles are superimposed).
				/*int mid_x = (sh_ur_x + sh_ll_x)/2;
				int mid_y = (sh_ur_y + sh_ll_y)/2;
				if(((mid_x > sv_ll_x) && (mid_x < sv_ur_x)) && ((mid_y > sv_ll_y) && (mid_y < sv_ur_y))){
					thirdPoint = -1;
				}*/
				if(thirdPoint == 0){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					na.travelTo((sh_ur_x+1)*SQUARE_LENGTH,(sh_ur_y)*SQUARE_LENGTH);
				}else if(thirdPoint == 1){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					na.travelTo((sh_ll_x-1)*SQUARE_LENGTH,(sh_ll_y+1)*SQUARE_LENGTH);
				}
			}

			//perform localization at the end -- need to make sure this works
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.midpointLocalization();
			odometer.setX(na.currentX);
			odometer.setY(na.currentY);
			odometer.setTheta(lightLoc.currentTheta);
			System.out.println("X localization at the end: " + odometer.getX());
			System.out.println("Y localization at the end: " + odometer.getY());
			System.out.println("Theta localization at the end: " + odometer.getTheta());
			
			if(na.currentX != sg_ur_x*SQUARE_LENGTH && na.currentY != sg_ur_y*SQUARE_LENGTH){
				while(!gotToDestination){
					gotToDestination = na.travelTo(na.currentX, sg_ur_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5, false,false);
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(sg_ur_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5, false,false);
				}
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
			Sound.playNote(Sound.FLUTE, 600, 300); 
			Sound.playNote(Sound.FLUTE, 600, 300); 
			Sound.playNote(Sound.FLUTE, 600, 300);
			
			//zipline
			gotToDestination = false;
			if(sg_ur_x != zo_g_x && sg_ur_y != zo_g_y){
				while(!gotToDestination){
					if (Math.abs(zo_g_x-sg_ur_x) > Math.abs(zo_g_y-sg_ur_y))
						gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, sg_ur_y*SQUARE_LENGTH);
					else 
						gotToDestination = na.travelTo(sg_ur_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5.8, false,false);
					System.out.println("lightLoc1");
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
				double cTheta = odometer.getTheta();
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5.0, false,true);
					while(na.isNavigating()){
					}
				}
				System.out.println("lightLoc2");
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
			na.makeTurn(3, false,false);
			odometer.setTheta(lightLoc.currentTheta);
			System.out.println("x before zipline: " + odometer.getX());
			System.out.println("y before zipline: " + odometer.getY());
			System.out.println("Theta before zipline: " + odometer.getTheta());
			if ((zc_g_x == zc_r_x) || (zc_g_y == zc_r_y)){
				na.doZipline(zc_g_x * SQUARE_LENGTH, zc_g_y * SQUARE_LENGTH, 3*ZIPLENGTH, 23);
				System.out.println("Zipline is aligned!");
			}
			else{
				na.doZipline(zc_g_x * SQUARE_LENGTH, zc_g_y * SQUARE_LENGTH, 3*ZIPLENGTH, 40);
				System.out.println("Zipline is not aligned!");
			}
			while(na.onZipline() || na.isNavigating()){
			}
			na.motorStop();
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
			}
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			na.makeTurn(3.8, false,false);
			double th = odometer.getTheta();
			odometer.setTheta(getThetaCorrection(th));
	//		System.out.println("TH is " + th);
			gotToDestination = false;
			if(zo_r_x != CORNERS[redCorner][0]*SQUARE_LENGTH && zo_r_y != CORNERS[redCorner][1]*SQUARE_LENGTH){
				while(!gotToDestination){
					gotToDestination = na.travelTo(CORNERS[redCorner][0]*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5, false,false);
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(CORNERS[redCorner][0]*SQUARE_LENGTH, CORNERS[redCorner][1]*SQUARE_LENGTH);
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5, false,false);
				}
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
		}
		else if (TEAM_NUMBER == greenTeam){
			na.makeTurn(2.7, false,false);
			odometer.setX(CORNERS[greenCorner][0]*SQUARE_LENGTH);
			odometer.setY(CORNERS[greenCorner][1]*SQUARE_LENGTH);
			odometer.setTheta(CORNERS[greenCorner][2]);
			boolean gotToDestination = false;
			if(CORNERS[greenCorner][0] != zo_g_x && CORNERS[greenCorner][1] != zo_g_y){
				while(!gotToDestination){
					if (Math.abs(zo_g_x-CORNERS[greenCorner][0]) > Math.abs(zo_g_y-CORNERS[greenCorner][1]))
						gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, CORNERS[greenCorner][1]*SQUARE_LENGTH);
					else 
						gotToDestination = na.travelTo(CORNERS[greenCorner][0]*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5.8, false,false);
					System.out.println("lightLoc1");
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(zo_g_x*SQUARE_LENGTH, zo_g_y*SQUARE_LENGTH);
				double cTheta = odometer.getTheta();
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5.0, false,true);
					while(na.isNavigating()){
					}
				}
				System.out.println("lightLoc2");
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
			na.makeTurn(3, false,false);
			odometer.setTheta(lightLoc.currentTheta);
			System.out.println("x before zipline: " + odometer.getX());
			System.out.println("y before zipline: " + odometer.getY());
			System.out.println("Theta before zipline: " + odometer.getTheta());
			if ((zc_g_x == zc_r_x) || (zc_g_y == zc_r_y)){
				na.doZipline(zc_g_x * SQUARE_LENGTH, zc_g_y * SQUARE_LENGTH, 3*ZIPLENGTH, 23);
				System.out.println("Zipline is aligned!");
			}
			else{
				na.doZipline(zc_g_x * SQUARE_LENGTH, zc_g_y * SQUARE_LENGTH, 3*ZIPLENGTH, 40);
				System.out.println("Zipline is not aligned!");
			}
			while(na.onZipline() || na.isNavigating()){
			}
			na.motorStop();
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
			}
			odometer.setX(zo_r_x*SQUARE_LENGTH);
			odometer.setY(zo_r_y*SQUARE_LENGTH);
			na.makeTurn(3.8, false,false);
			double th = odometer.getTheta();
			odometer.setTheta(getThetaCorrection(th));
			System.out.println("TH is " + th);
			gotToDestination = false;
			if(zo_r_x != sr_ur_x && zo_r_y != sr_ur_y){
				while(!gotToDestination){
					if (zo_r_x != zc_r_x)
						gotToDestination = na.travelTo(zo_r_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
					else if(zo_r_y != zc_r_y)
						gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, zo_r_y*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5, false,false);
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(sr_ur_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5, false,false);
				}
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
			na.travelTo(sr_ur_x*SQUARE_LENGTH, sr_ur_y*SQUARE_LENGTH);
			while(na.isNavigating()){
			}
			double curTheta = odometer.getTheta();
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.doLightLocalization();
			while(na.isNavigating()){
			}
			na.makeTurn(5, false,false);
			odometer.setX(sr_ur_x*SQUARE_LENGTH);
			odometer.setY(sr_ur_y*SQUARE_LENGTH);
			odometer.setTheta(curTheta-90);
			
			Sound.playNote(Sound.FLUTE, 600, 300); 
			Sound.playNote(Sound.FLUTE, 600, 300); 
			Sound.playNote(Sound.FLUTE, 600, 300); 

			//SHALLOW CROSSING
			System.out.println("X start: " + odometer.getX());
			System.out.println("Y start: " + odometer.getY());
			System.out.println("theta start: " + odometer.getTheta());

			double centerRed_x = Math.abs(red_ur_x+red_ll_x)/2;
			double centerRed_y = Math.abs(red_ur_y+red_ll_y)/2;
			System.out.println("center " + centerRed_x + ";" + centerRed_y);
			double[] distances = new double[4];//0: sh_ll / 1: sh_ur / 2:sv_ll / 3:sv_ur
			int thirdPoint;
			gotToDestination = false;
			distances[0] = Math.sqrt(Math.pow(sh_ll_y-centerRed_y,2) + Math.pow(sh_ll_x-centerRed_x,2));
			distances[1] = Math.sqrt(Math.pow(sh_ur_y-centerRed_y,2) + Math.pow(sh_ur_x-centerRed_x,2));
			distances[2] = Math.sqrt(Math.pow(sv_ll_y-centerRed_y,2) + Math.pow(sv_ll_x-centerRed_x,2));
			distances[3] = Math.sqrt(Math.pow(sv_ur_y-centerRed_y,2) + Math.pow(sv_ur_x-centerRed_x,2));
			for(int k=0;k<4;k++){
				System.out.println("distance " + k + " - " + distances[k]);
			}
			int firstPoint = getMinDistance(distances);
			System.out.println(firstPoint);
			System.out.println("test print");
			if(firstPoint == 0 || firstPoint == 1){
				if(firstPoint == 0){
					System.out.println("going to sh_ll");
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					System.out.println("X first point: " + odometer.getX());
					System.out.println("Y first point: " + odometer.getY());
					System.out.println("theta first point: " + odometer.getTheta());
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					System.out.println("X second point: " + odometer.getX());
					System.out.println("Y second point: " + odometer.getY());
					System.out.println("theta second point: " + odometer.getTheta());
				}else if(firstPoint == 1){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
				}
				distances = new double[2];
				distances[0] = Math.sqrt(Math.pow(sv_ll_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sv_ll_x-(odometer.getX()/SQUARE_LENGTH),2));
				distances[1] = Math.sqrt(Math.pow(sv_ur_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sv_ur_x-(odometer.getX()/SQUARE_LENGTH),2));
				thirdPoint = getMinDistance(distances);

				//if the point where you go is behind you (in case the two rectangles are superimposed).
				/*int mid_x = (sh_ur_x + sh_ll_x)/2;
				int mid_y = (sh_ur_y + sh_ll_y)/2;
				if(((mid_x > sv_ll_x) && (mid_x < sv_ur_x)) && ((mid_y > sv_ll_y) && (mid_y < sv_ur_y))){
					thirdPoint = -1;
				}*/
				System.out.println("X point: " + odometer.getX());
				System.out.println("Y point: " + odometer.getY());
				System.out.println("theta point: " + odometer.getTheta());
				System.out.println("distance 0 " + distances[0]);
				System.out.println("distance 1 " + distances[1]);
				System.out.println(thirdPoint);
				if(thirdPoint == 0){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					na.travelTo((sv_ur_x-1)*SQUARE_LENGTH,(sv_ur_y+1)*SQUARE_LENGTH);
				}else if(thirdPoint == 1){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
						//lightLoc.midpointLocalization();
						//odometer.setX(na.currentX);
						//odometer.setY(na.currentY);
						//odometer.setTheta(lightLoc.currentTheta);
					}
					gotToDestination = false;
					na.travelTo((sv_ll_x+1)*SQUARE_LENGTH,(sv_ll_y-1)*SQUARE_LENGTH);
				}
			}

			if(firstPoint == 2 || firstPoint == 3){
				if(firstPoint == 2){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
				}else if(firstPoint == 3){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ur_x-0.5)*SQUARE_LENGTH,(sv_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sv_ll_x+0.5)*SQUARE_LENGTH,(sv_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
				}
				distances = new double[2];
				distances[0] = Math.sqrt(Math.pow(sh_ll_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sh_ll_x-(odometer.getX()/SQUARE_LENGTH),2));
				distances[1] = Math.sqrt(Math.pow(sh_ur_y-(odometer.getY()/SQUARE_LENGTH),2) + Math.pow(sh_ur_x-(odometer.getX()/SQUARE_LENGTH),2));
				thirdPoint = getMinDistance(distances);

				//if the point where you go is behind you (in case the two rectangles are superimposed).
				/*int mid_x = (sh_ur_x + sh_ll_x)/2;
				int mid_y = (sh_ur_y + sh_ll_y)/2;
				if(((mid_x > sv_ll_x) && (mid_x < sv_ur_x)) && ((mid_y > sv_ll_y) && (mid_y < sv_ur_y))){
					thirdPoint = -1;
				}*/
				if(thirdPoint == 0){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					na.travelTo((sh_ur_x+1)*SQUARE_LENGTH,(sh_ur_y)*SQUARE_LENGTH);
				}else if(thirdPoint == 1){
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ur_x-0.5)*SQUARE_LENGTH,(sh_ur_y-0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					while(!gotToDestination){
						gotToDestination = na.travelTo((sh_ll_x+0.5)*SQUARE_LENGTH,(sh_ll_y+0.5)*SQUARE_LENGTH);
					}
					gotToDestination = false;
					na.travelTo((sh_ll_x-1)*SQUARE_LENGTH,(sh_ll_y+1)*SQUARE_LENGTH);
				}
			}

			//perform localization at the end -- need to make sure this works
			na.securityTurn();
			lightLoc = new LightLocalizer(odometer, na);
			lightLoc.midpointLocalization();
			odometer.setX(na.currentX);
			odometer.setY(na.currentY);
			odometer.setTheta(lightLoc.currentTheta);
			System.out.println("X localization at the end: " + odometer.getX());
			System.out.println("Y localization at the end: " + odometer.getY());
			System.out.println("Theta localization at the end: " + odometer.getTheta());
			leftMotor.stop();
			rightMotor.stop();
			
			//GO BACK TO START ZONE
			if(na.currentX != CORNERS[greenCorner][0]*SQUARE_LENGTH && na.currentY != CORNERS[greenCorner][1]*SQUARE_LENGTH){
				while(!gotToDestination){
					gotToDestination = na.travelTo(na.currentX, CORNERS[greenCorner][1]*SQUARE_LENGTH);
					na.securityTurn();
					lightLoc = new LightLocalizer(odometer, na);
					lightLoc.midpointLocalization();
					na.makeTurn(5, false,false);
					odometer.setX(na.currentX);
					odometer.setY(na.currentY);
					odometer.setTheta(lightLoc.currentTheta);
				}
				gotToDestination = false;
			}
			while(!gotToDestination){
				gotToDestination = na.travelTo(CORNERS[greenCorner][0]*SQUARE_LENGTH, CORNERS[greenCorner][1]*SQUARE_LENGTH);
				na.securityTurn();
				lightLoc = new LightLocalizer(odometer, na);
				lightLoc.midpointLocalization();
				if(!gotToDestination) {
					na.makeTurn(5, false,false);
				}
				odometer.setX(na.currentX);
				odometer.setY(na.currentY);
				odometer.setTheta(lightLoc.currentTheta);
			}
		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}

	private static int getMinDistance(double[] distances){
		double curMin = distances[0];
		int index = 0;
		for(int i=1; i<distances.length;i++){
			if(distances[i] < curMin){
				curMin = distances[i];
				index = i;
			}
		}
		return index;
	}

	private static double getThetaCorrection(double theta){
		if(Math.abs(theta) < 15) return 0;
		else if (Math.abs(theta-90) < 15) return 90;
		else if (Math.abs(theta) < 15) return 180;
		else if (Math.abs(theta) < 15) return 270;
		else 
			return theta;
	}

}
