/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4512.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */


/*
 * I/O MAP
 * - PWM		 Motor Name					Motor Type
 *    0 	Right Collector Motor			  Victor
 *    1 	Left Collector Motor			  Victor
 *    2 	Climb Motors					  Victor
 *    3 									  Unused
 *    4 	Lift Motors						  Victor
 *    5   									  Unused
 *    6 	Hook Motor						  Victor
 *    7 									  Unused
 *    8 	Left Drive Motors				  Spark
 *    9 	Right Drive Motors				  Spark
 * 
 * - DIO		Sensor Name					Sensor Type
 *    0		Reed Switch Top					Reed Switch
 *    1		Lift Encoder Port A				  Encoder
 *    2		Lift Encoder Port B				  Encoder
 *    3		Reed Switch Bottom				Reed Switch
 *    4		Left Drive Encoder Port A		  Encoder
 *    5		Left Drive Encoder Port B		  Encoder
 *    6 	Right Drive Encoder Port A		  Encoder
 *    7     Right Drive Encoder Port B        Encoder
 *    8
 *    9
*/

public class Robot extends IterativeRobot {
	//Buffers to Hold Ultrasonic Information From Rioduino
	byte[] getBuffer = new byte[8];
	byte[] sendBuffer = new byte[8];
	
	//Ultrasonic Sensor Values
	long frontLeft = 0;
	long frontRight = 0;
	long left = 0;
	long right = 0;
	
	//Auto
	private static final String kDefaultAuto = "Cross Auto Line";
	private static final String kRightAuto = "Right Auto";
	private static final String kLeftSideTurn = "Far Left Auto";
	private static final String kRightSideTurn = "Far Right Auto";
	private static final String kLeftAuto = "Encoder Auto Test";
	private static final String kNothingAuto = "Literally Nothing";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	private String gameData;
	private Timer timer;
	private char switch1;
	
	//Auto Position Checks
	private boolean lifted1 = false, 
					movedForward1 = false, 
					turned = false, 
					movedForward2 = false, 
					lifted2 = false, 
					placedBlock = false;
	
	//Limelight Variables
	NetworkTable table;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry camMode;
	NetworkTableEntry ledMode;
	double x;
	double y;
	double area;
	
	//Declare Rioduino
	I2C rioduino = new I2C(I2C.Port.kMXP, 0x08);
	private boolean failFlag;
	
	//Drive Motors & Encoder
	public SpeedController driveLeft;
	public SpeedController driveRight;
	public Encoder driveEncoderLeft;
	public Encoder driveEncoderRight;
	
	//Collector Motors
	public SpeedController collectorLeft;
	public SpeedController collectorRight;
		
	//Lift Motors & Encoder
	public SpeedController liftMotors;
	public Encoder liftEncoder;
	private final int footCount = 1303;
	
	//Climb Motors
	public SpeedController climbMotors;
	public SpeedController hookMotor;
	
	//Reed Switches
	DigitalInput reedSwitchTop;
	DigitalInput reedSwitchBot;
	
	//Joysticks & Joystick Values
	public Joystick joystickLeft;
	public Joystick joystickRight;
    public double joystickLeftValue = 0.0;
    public double joystickRightValue = 0.0;
    public double joystickRightTwist = 0.0;
    final double maxChange = 0.05;
    public double change = 0.0;
	
    //Speed Values
    public double driveSpeed = 0.9;
    final double collectSpeed = -1.0;
    final double liftSpeed = 1.0;
    final double climbSpeed = 1.0;
    final double deadZone = 0.05;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		//Adds Auto Choices to Dashboard
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("Left Auto", kLeftAuto);
		m_chooser.addObject("Right Auto", kRightAuto);
		//m_chooser.addObject("Far Left Auto", kFarLeftAuto);
		//m_chooser.addObject("Far Right Auto", kFarRightAuto);
		m_chooser.addObject("Encoder Auto Test", kLeftAuto);
		m_chooser.addObject("Literally Nothing", kNothingAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
				
		//Assigns Motors to PWM Slots
		collectorRight = new Victor(0);
		collectorLeft = new Victor(1);
		climbMotors = new Victor(2);
		liftMotors = new Victor(4);
		hookMotor = new Victor(6);
		driveLeft = new Spark(8);
		driveRight = new Spark(9);
		
		//Assign Sensors to DIO Slots
		reedSwitchTop = new DigitalInput(0);
		reedSwitchBot = new DigitalInput(3);
		
		//Assigns Encoders to DIO Slots, Resets Them, and Prints Out Their Initial Coutn and Rate
		liftEncoder = new Encoder(1, 2);
		driveEncoderLeft = new Encoder(4, 5);
		driveEncoderRight = new Encoder(6, 7);
		liftEncoder.reset();
		driveEncoderLeft.reset();
		driveEncoderRight.reset();
		
    	SmartDashboard.putString("Lift Count", "" + liftEncoder.get());
    	SmartDashboard.putString("Lift Rate", "" + liftEncoder.getRate());
    	SmartDashboard.putString("Left Drive Count", "" + -driveEncoderLeft.get());
    	SmartDashboard.putString("Left Drive Rate", "" + -driveEncoderLeft.getRate());
    	SmartDashboard.putString("Right Drive Count", "" + driveEncoderRight.get());
    	SmartDashboard.putString("Right Drive Rate", "" + driveEncoderRight.getRate());
    	
		//Assigns Joysticks to USB Slots
		joystickLeft = new Joystick(1);
		joystickRight = new Joystick(0);
		
		//Gets Limelight Data From The Limelight and Puts It Onto A NetworkTable
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//Chooser for Auto
		m_autoSelected = m_chooser.getSelected();
		System.out.println("Auto selected: " + m_autoSelected);
		
		//Determine Switch Side & Add It To The Dashboard
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		switch1 = gameData.charAt(0);
		SmartDashboard.putString("Switch Side", "" + switch1);
		
		//Start Timer and Reset All Motors
		timer = new Timer();
		reset();
		
		//Resets Rioduino Buffers
		for (byte i = 0; i < 8; i++) {
			sendBuffer[i] = i;
			getBuffer[i] = 0;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		//Get Sensor Values Through the Rioduino
		failFlag = rioduino.transaction(sendBuffer, 8, getBuffer, 8);
		
		if (failFlag) {
			SmartDashboard.putString("Rioduino Status", "Bad");
		} else {
			SmartDashboard.putString("Rioduino Status", "Good");
			
			for (byte i = 0; i < getBuffer.length; i++) {
					getBuffer[i] = (byte)getBuffer[i];
			}
			
			//Doesn't Work. Will Fix
			frontLeft = ((getBuffer[0] << 8) & 0xFF00) | (getBuffer[1] & 0xFF);
			frontRight = ((getBuffer[2] << 8) & 0xFF00) | (getBuffer[3] & 0xFF);
			left = ((getBuffer[4] << 8) & 0xFF00) | (getBuffer[5] & 0xFF);
			right = ((getBuffer[6] << 8) & 0xFF00) | (getBuffer[7] & 0xFF);
			
			SmartDashboard.putString("Front Left", "" + frontLeft); 
			SmartDashboard.putString("Front Right", "" + frontRight); 
			SmartDashboard.putString("Left", "" + left); 
			SmartDashboard.putString("Right", "" + right); 
		}
		
		//Assign Limelight Variables
		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		camMode = table.getEntry("camMode");
		ledMode = table.getEntry("ledMode");
		x = tx.getDouble(0);
		y = ty.getDouble(0);
		area = ta.getDouble(0);
		camMode.setNumber(0);
		ledMode.setNumber(1);
		
		//Run Selected Auto
		switch (m_autoSelected) {
			case kLeftAuto:
				if (switch1 == 'L') {
					collectorMotors(collectSpeed * 0.25);
				
					while (!lifted1) {
						liftMotors.set(liftSpeed);
						if (liftEncoder.get() > 3600) {
							liftMotors.set(0.0);
							lifted1 = true;
						}
					}
				
					while (lifted1 && !movedForward1) {
						tankDrive(0.7, 0.7);
						if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 9.75 * footCount) {
							tankDrive(0.0, 0.0);
							movedForward1 = true;
						}
					}
				
					while (movedForward1 && !placedBlock) {	
						timer.start();
						collectorMotors(-collectSpeed);
						if (timer.get() > 3.0) {
							collectorMotors(0.0);
							timer.stop();
							placedBlock = true;
						}
					}
				} else {
					while (!movedForward1) {
						tankDrive(0.7, 0.7);
						if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 20000) {
							movedForward1 = true;
						}
					}
				}
				reset();
				break;
			case kLeftSideTurn: 
			//If positioned on the left side of the field & not in direct line of sight of the switch
				if (switch1 == 'L') {
					collectorMotors(collectSpeed * 0.25);
				
					while (!lifted1) {
						liftMotors.set(liftSpeed);
						if (liftEncoder.get() > 3600) {
							liftMotors.set(0.0);
							lifted1 = true;
						}
					}
				
					while (lifted1 && !movedForward1) {
						tankDrive(0.7, 0.7);
						if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 9.75 * footCount) {
							tankDrive(0.0, 0.0);
							movedForward1 = true;
						}
					}
					
					driveEncoderLeft.reset();
					driveEncoderRight.reset();
					
					while (movedForward1 && !turned) {
						tankDrive(0.6, -0.6);
						if ((Math.abs(driveEncoderLeft.get()) + Math.abs(driveEncoderRight.get())) / 2.0 > 2 * footCount) {
							tankDrive(0.0, 0.0);
							turned = true;
						}
					}
				
					while (movedForward2 && !placedBlock) {	
						timer.start();
						collectorMotors(-collectSpeed);
						if (timer.get() > 3.0) {
							collectorMotors(0.0);
							timer.stop();
							placedBlock = true;
						}
					}
				} else {
					while (!movedForward1) {
						tankDrive(0.7, 0.7);
						if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 20000) {
							movedForward1 = true;
						}
					}
				}
				reset();
				break;
			case kRightSideTurn:
			//If positioned on the right side of the field & not in direct line of sight of the switch
				if (switch1 == 'R') {
					collectorMotors(collectSpeed * 0.25);
					
					while (!lifted1) {
						liftMotors.set(liftSpeed);
						if (liftEncoder.get() > 3000) {
							liftMotors.set(0.0);
							lifted1 = true;
						}
					}
					
					while (lifted1 && !movedForward1) {
						tankDrive(0.7, 0.7);
						if (getBuffer[5] < 7 && getBuffer[5] > 0) {
							tankDrive(0.0, 0.0);
							movedForward1 = true;
						}
					}
					
					while (movedForward1 && !turned) {
						tankDrive(-0.7, 0.7);
						if (getBuffer[1] > 0 && getBuffer[3] > 0) {
							tankDrive(0.0, 0.0);
							turned = true;
						}
					}
					
					while (turned && !movedForward2) {
						tankDrive(0.7, 0.7);
						if (getBuffer[1] < 10 && getBuffer[3] < 10 && getBuffer[1] > 0 && getBuffer[3] > 0) {
							tankDrive(0.0, 0.0);
							turned = true;
						}
					}
					
					while (movedForward2 && !lifted2) {
						liftMotors.set(liftSpeed);
						if (liftEncoder.get() > 6000) {
							liftMotors.set(0.0);
							lifted2 = true;
						}
					}
					
					while (lifted2 && !placedBlock) {	
						timer.start();
						collectorMotors(-collectSpeed);
						if (timer.get() > 3.0) {
							collectorMotors(0.0);
							placedBlock = true;
						}
					}
				} else {
					timer.start();
					
					if (timer.get() >= 0.0 && timer.get() <= 5.0) {
						tankDrive(0.7, 0.7);
					} else {
						tankDrive(0.0, 0.0);
					}
					
					timer.stop();
				}

				reset();
				break;
			case kRightAuto:
			//If positioned on the right side of the field & in direct line of sight of the switch
				if (switch1 == 'L') {
					collectorMotors(collectSpeed * 0.25);
				
					while (!lifted1) {
						liftMotors.set(liftSpeed);
						if (liftEncoder.get() > 3600) {
							liftMotors.set(0.0);
							lifted1 = true;
						}
					}
				
					while (lifted1 && !movedForward1) {
						tankDrive(0.7, 0.7);
						if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 9.75 * footCount) {
							tankDrive(0.0, 0.0);
							movedForward1 = true;
						}
					}
				
					while (movedForward1 && !placedBlock) {	
						timer.start();
						collectorMotors(-collectSpeed);
						if (timer.get() > 3.0) {
							collectorMotors(0.0);
							timer.stop();
							placedBlock = true;
						}
					}
				} else {
					while (!movedForward1) {
						tankDrive(0.7, 0.7);
						if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 20000) {
							movedForward1 = true;
						}
					}
				}
				reset();
				break;
			case kNothingAuto:
			//Do Nothing
				reset();
				break;
			case kDefaultAuto:
			//Cross Auto Line
			default:
				while (!movedForward1) {
					tankDrive(0.7, 0.7);
					if ((-driveEncoderLeft.get() + driveEncoderRight.get()) / 2.0 >= 20000) {
						movedForward1 = true;
					}
				}
				
				/*
				timer.start();
				
				if (timer.get() >= 0.0 && timer.get() <= 5.0) {
					tankDrive(0.7, 0.7);
				} else {
					tankDrive(0.0, 0.0);
				}

				reset();
				timer.stop();
				*/
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//Assigns Joystick Value To The Joysticks' Y-Axis
				//joystickLeftValue = joystickLeft.getY();
				//joystickRightValue = joystickRight.getY();
				//joystickRightTwist = joystickRight.getTwist();
						
				//Slew Rate Limited Joystick Values
				change = joystickLeft.getY() - joystickLeftValue;
				if (change > maxChange) change = maxChange;
				else if (change <= maxChange) change = -maxChange;
				joystickLeftValue += change;
						
				change = joystickRight.getY() - joystickRightValue;
				if (change > maxChange) change = maxChange;
				else if (change <= maxChange) change = -maxChange;
				joystickRightValue += change;
				
				change = joystickRight.getTwist() - joystickRightTwist;
				if (change > maxChange) change = maxChange;
				else if (change <= maxChange) change = -maxChange;
				joystickRightTwist += change;
				
				/*
		    	//Apply Deadzone on Joysticks
		    	if (joystickLeftValue <= deadZone && joystickLeftValue >= -deadZone) {
		    		joystickLeftValue = 0;
		    	} 
				    	
		    	if (joystickRightValue <= deadZone && joystickRightValue >= -deadZone) {
		    		joystickRightValue = 0;
		    	}
				
		    	if (joystickRightTwist <= deadZone * 2 && joystickRightTwist >= -deadZone * 2) {
		    		joystickRightTwist = 0;
		    	}
		    	*/
		    	
		    	//Slow Mode Code
		    	if (driveSpeed == 0.9 && joystickLeft.getRawButton(2)) {
		    		driveSpeed = 0.45;
		    	} else if (driveSpeed == 0.45 && joystickLeft.getRawButton(2)) {
		    		driveSpeed = 0.9;
		    	}
				    	
		    	SmartDashboard.putString("Drive Speed", "" + driveSpeed);
				    	
		    	//Collector Controls
		    	if (joystickRight.getTrigger()) {
		    		collectorMotors(collectSpeed);
		    	} else if (joystickLeft.getTrigger()) {
		    		collectorMotors(-collectSpeed);
		    	} else {
		    		collectorMotors(collectSpeed * 0.22);
		    	}
				    	
		    	//Reed Switch and Lift Controls
		    	SmartDashboard.putBoolean("Reed Switch Top", reedSwitchTop.get());
		    	SmartDashboard.putBoolean("Reed Switch Bot", reedSwitchBot.get());
		    	
		    	if (!reedSwitchBot.get())
		    		liftEncoder.reset();
		    	
		    	if (joystickRight.getRawButton(5) && reedSwitchTop.get() && liftEncoder.get() >= 20000) {
		    		liftMotors.set(liftSpeed * 0.7);
		    	} else if (joystickRight.getRawButton(5) && reedSwitchTop.get()) {
		    		liftMotors.set(liftSpeed);
				} else if (joystickRight.getRawButton(3) && liftEncoder.get() <= 3000) {
		    		liftMotors.set(-liftSpeed * 0.5);
		    	} else if (joystickRight.getRawButton(3)) {
		    		liftMotors.set(-liftSpeed * 0.8);
		    	} else {
		    		liftMotors.set(0.0);
		    	}
				    	
		    	//Encoder Tests
		    	SmartDashboard.putString("Lift Count", "" + liftEncoder.get());
		    	SmartDashboard.putString("Lift Rate", "" + liftEncoder.getRate());
		    	SmartDashboard.putString("Left Drive Count", "" + -driveEncoderLeft.get());
		    	SmartDashboard.putString("Left Drive Rate", "" + -driveEncoderLeft.getRate());
		    	SmartDashboard.putString("Right Drive Count", "" + driveEncoderRight.get());
		    	SmartDashboard.putString("Right Drive Rate", "" + driveEncoderRight.getRate());
		    	
		    	//Hook Controls
		    	if (joystickLeft.getRawButton(6)) {
		    		hookMotor.set(climbSpeed);
		    	} else if (joystickLeft.getRawButton(4)) {
		    		hookMotor.set(-climbSpeed);
		    	} else {
		    		hookMotor.set(0.0);
		    	}
		    	
		    	//Climber Controls
		    	if (joystickLeft.getRawButton(3)) {
		    		climbMotors.set(climbSpeed);
		    	} else {
		    		climbMotors.set(0.0);
		    	}
		    	
		    	//Tank Drive
		    	if (joystickRight.getRawButton(2)) {
		    		if (joystickRightTwist != 0.0) {
		    			tankDrive(-joystickRightTwist * 0.7, joystickRightTwist * 0.7);
		    		} else if (joystickRightValue != 0.0) {
		    			tankDrive(joystickRightValue, joystickRightValue);
		    		} else {
		    			tankDrive(0.0, 0.0);
		    		}
		    	} else {
		    		tankDrive(joystickLeftValue, joystickRightValue);
		    	}
				    	
		    	//Control Limelight LED's
		    	ledMode = table.getEntry("ledMode");
		    	
		    	if (joystickRight.getRawButton(7))
		    		ledMode.setNumber(1);
		    	else if (joystickRight.getRawButton(9))
		    		ledMode.setNumber(0);
		    	else if (joystickRight.getRawButton(11))
		    		ledMode.setNumber(2);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		//Assigns Joystick Value To The Joysticks' Y-Axis
		//joystickLeftValue = joystickLeft.getY();
		//joystickRightValue = joystickRight.getY();
		//joystickRightTwist = joystickRight.getTwist();
				
		//Slew Rate Limited Joystick Values
		change = joystickLeft.getY() - joystickLeftValue;
		if (change > maxChange) change = maxChange;
		else if (change <= maxChange) change = -maxChange;
		joystickLeftValue += change;
				
		change = joystickRight.getY() - joystickRightValue;
		if (change > maxChange) change = maxChange;
		else if (change <= maxChange) change = -maxChange;
		joystickRightValue += change;
		
		change = joystickRight.getTwist() - joystickRightTwist;
		if (change > maxChange) change = maxChange;
		else if (change <= maxChange) change = -maxChange;
		joystickRightTwist += change;
		
		/*
    	//Apply Deadzone on Joysticks
    	if (joystickLeftValue <= deadZone && joystickLeftValue >= -deadZone) {
    		joystickLeftValue = 0;
    	} 
		    	
    	if (joystickRightValue <= deadZone && joystickRightValue >= -deadZone) {
    		joystickRightValue = 0;
    	}
		
    	if (joystickRightTwist <= deadZone * 2 && joystickRightTwist >= -deadZone * 2) {
    		joystickRightTwist = 0;
    	}
    	*/
    	
    	//Slow Mode Code
    	if (driveSpeed == 0.9 && joystickLeft.getRawButton(2)) {
    		driveSpeed = 0.45;
    	} else if (driveSpeed == 0.45 && joystickLeft.getRawButton(2)) {
    		driveSpeed = 0.9;
    	}
		    	
    	SmartDashboard.putString("Drive Speed", "" + driveSpeed);
		    	
    	//Collector Controls
    	if (joystickRight.getTrigger()) {
    		collectorMotors(collectSpeed);
    	} else if (joystickLeft.getTrigger()) {
    		collectorMotors(-collectSpeed);
    	} else {
    		collectorMotors(collectSpeed * 0.22);
    	}
		    	
    	//Reed Switch and Lift Controls
    	SmartDashboard.putBoolean("Reed Switch Top", reedSwitchTop.get());
    	SmartDashboard.putBoolean("Reed Switch Bot", reedSwitchBot.get());
    	
    	if (!reedSwitchBot.get())
    		liftEncoder.reset();
    	
    	if (joystickRight.getRawButton(5) && reedSwitchTop.get() && liftEncoder.get() >= 20000) {
    		liftMotors.set(liftSpeed * 0.7);
    	} else if (joystickRight.getRawButton(5) && reedSwitchTop.get()) {
    		liftMotors.set(liftSpeed);
		} else if (joystickRight.getRawButton(3) && reedSwitchBot.get() && liftEncoder.get() > 3000) {
    		liftMotors.set(-liftSpeed * 0.5);
    	} else if (joystickRight.getRawButton(3) && reedSwitchBot.get()) {
    		liftMotors.set(-liftSpeed * 0.8);
    	} else {
    		liftMotors.set(0.0);
    	}
		    	
    	//Encoder Tests
    	SmartDashboard.putString("Lift Count", "" + liftEncoder.get());
    	SmartDashboard.putString("Lift Rate", "" + liftEncoder.getRate());
    	SmartDashboard.putString("Left Drive Count", "" + driveEncoderLeft.get());
    	SmartDashboard.putString("Left Drive Rate", "" + driveEncoderLeft.getRate());
    	SmartDashboard.putString("Right Drive Count", "" + driveEncoderRight.get());
    	SmartDashboard.putString("Right Drive Rate", "" + driveEncoderRight.getRate());
    	
    	//Hook Controls
    	if (joystickLeft.getRawButton(6)) {
    		hookMotor.set(climbSpeed * 0.8);
    	} else if (joystickLeft.getRawButton(4)) {
    		hookMotor.set(-climbSpeed * 0.5);
    	} else {
    		hookMotor.set(0.0);
    	}
    	
    	//Climber Controls
    	if (joystickLeft.getRawButton(3)) {
    		climbMotors.set(climbSpeed);
    	} else {
    		climbMotors.set(0.0);
    	}
    	
    	//Tank Drive
    	if (joystickRight.getRawButton(2)) {
    		if (joystickRightTwist != 0.0) {
    			tankDrive(joystickRightTwist, -joystickRightTwist);
    		} else if (joystickRightValue != 0.0) {
    			tankDrive(joystickRightValue, joystickRightValue);
    		} else {
    			tankDrive(0.0, 0.0);
    		}
    	} else {
    		tankDrive(joystickLeftValue, joystickRightValue);
    	}
		    	
    	//Control Limelight LED's
    	ledMode = table.getEntry("ledMode");
    	
    	if (joystickRight.getRawButton(7))
    		ledMode.setNumber(1);
    	else if (joystickRight.getRawButton(9))
    		ledMode.setNumber(0);
    	else if (joystickRight.getRawButton(11))
    		ledMode.setNumber(2);
	}

	//Tank Drive
	public void tankDrive(double leftSpeed, double rightSpeed) {
		driveLeft.set(-leftSpeed * driveSpeed);
		driveRight.set(rightSpeed * driveSpeed);
	}
	
	//Collector Motors
	public void collectorMotors(double speed) {
		collectorLeft.set(speed);
		collectorRight.set(-speed);
	}
	
	//Resets Motors
	public void reset() {
		tankDrive(0.0, 0.0);
		collectorMotors(0.0);
		liftMotors.set(0.0);
		climbMotors.set(0.0);
		hookMotor.set(0.0);
	}
}