// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private enum TestType {
        QUASISTATIC,
        DYNAMIC,
        OTHER
    }

    private final CANSparkMax leftLeader = new CANSparkMax(Constants.LEFT_FRONT_PORT, MotorType.kBrushless);
    private final CANSparkMax rightLeader = new CANSparkMax(Constants.RIGHT_FRONT_PORT, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final ArrayList<Double> data = new ArrayList<>(Constants.DATA_VECTOR_SIZE);
    private double voltage = 0.0;
    private double leftVoltage = 0.0;
    private double rightVoltage = 0.0;
    private double currentTime = 0.0;

    private boolean rotate = false;
    private TestType testType = TestType.QUASISTATIC;
    private double voltageCommand = .25;
    private double startTime = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double kS = 0.0;
    private boolean avg = false;
    private double velocity;

    public Robot() {
        super(0.005);
        //CANSparkMax leftFront = new CANSparkMax(Constants.LEFT_FRONT_PORT, MotorType.kBrushless);
        CANSparkMax leftBack = new CANSparkMax(Constants.LEFT_BACK_PORT, MotorType.kBrushless);

        leftLeader.restoreFactoryDefaults();
        //leftFront.restoreFactoryDefaults();
        leftBack.restoreFactoryDefaults();

        leftLeader.setInverted(Constants.LEFT_INVERTED);
        //leftFront.follow(leftLeader);
        leftBack.follow(leftLeader);

        leftEncoder.setPositionConversionFactor(Constants.DISTANCE_PER_ROTATION);
        leftEncoder.setVelocityConversionFactor(Constants.DISTANCE_PER_ROTATION / 60);


        //CANSparkMax rightFront = new CANSparkMax(Constants.RIGHT_FRONT_PORT, MotorType.kBrushless);
        CANSparkMax rightBack = new CANSparkMax(Constants.RIGHT_BACK_PORT, MotorType.kBrushless);

        rightLeader.restoreFactoryDefaults();
        //rightFront.restoreFactoryDefaults();
        rightBack.restoreFactoryDefaults();

        rightLeader.setInverted(Constants.RIGHT_INVERTED);
        //rightFront.follow(rightLeader);
        rightBack.follow(rightLeader);

        rightEncoder.setPositionConversionFactor(Constants.DISTANCE_PER_ROTATION);
        rightEncoder.setVelocityConversionFactor(Constants.DISTANCE_PER_ROTATION / 60);

        LiveWindow.disableAllTelemetry();
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {}
    
    
    /**
     * This method is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}
    
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        // Initialize logging
        /*String testTypeString = Shuffleboard.getString("SysIdTestType", "");
        if (testTypeString.equals("Quasistatic")) {
            testType = TestType.QUASISTATIC;
        } else if (testTypeString.equals("Dynamic")) {
            testType = TestType.DYNAMIC;
        } else {
            testType = TestType.OTHER;
        }*/

        //rotate = SmartDashboard.getBoolean("SysIdRotate", false);
        //voltageCommand = SmartDashboard.getNumber("SysIdVoltageCommand", 0.0);
        startTime = Timer.getFPGATimestamp();
        data.clear();
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        velocity = leftEncoder.getVelocity()*((2 * 0.0508/*wheel radius*/ * Math.PI) / (8.142857/*gear ratio*/ * 60));
        logData(leftEncoder.getPosition(), rightEncoder.getPosition(), velocity, rightEncoder.getVelocity(), Math.toRadians(gyro.getAngle()), Math.toRadians(gyro.getRate()));
 
        SmartDashboard.putNumber("Velocity", velocity);

        if (velocity > 0.001 && kS == 0) {
            kS = leftVoltage;
        }

        if (kS != 0){
            kV = (leftVoltage - kS) / velocity;
        }

        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);

        SmartDashboard.putNumber("kV", kV);
        SmartDashboard.putNumber("kS", kS);
    }


    /** This method is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        pushNTDiagnostics();
    }
    
    
    /** This method is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        leftLeader.setVoltage(0.0);
        rightLeader.setVoltage(0.0);
        sendData();
    }


    /** This method is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        pushNTDiagnostics();
    }
    
    
    /** This method is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        pushNTDiagnostics();
    }

    private void updateData() {
        currentTime = Timer.getFPGATimestamp();
        if (testType == TestType.QUASISTATIC) {
            voltage = voltageCommand * (currentTime - startTime);
        } else if (testType == TestType.DYNAMIC) {
            voltage = voltageCommand;
        } else {
            voltage = 0.0;
        }
    }

    private void logData(double leftPosition, double rightPosition, double leftRate, double rightRate, double gyroPosition, double gyroRate) {
        updateData();
        if (data.size() < Constants.DATA_VECTOR_SIZE) {
            //data.add(currentTime);
            data.add(leftVoltage);
            //data.add(rightVoltage);
            //data.add(leftPosition);
            //data.add(rightPosition);
            data.add(leftRate);
            //data.add(rightRate);
            //data.add(gyroPosition);
            //data.add(gyroRate);
        }
        leftVoltage = voltage * (rotate ? -1 : 1);
        rightVoltage = voltage;
    }

    private void sendData() {
        SmartDashboard.putBoolean("SysIdOverflow", data.size() > Constants.DATA_VECTOR_SIZE);

        String dataString = data.toString();
        SmartDashboard.putString("SysIdTelemetry", data.toString().substring(1, dataString.length() - 1));

        reset();
    }

    private void reset() {
        voltage = 0.0;
        currentTime = 0.0;
        startTime = 0.0;
        leftVoltage = 0.0;
        rightVoltage = 0.0;
        avg = false;
        data.clear();
    }



    private void pushNTDiagnostics() {
        SmartDashboard.putNumber("Left Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Position", rightEncoder.getPosition());
        SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
        SmartDashboard.putNumber("Gyro Reading", Math.toRadians(gyro.getAngle()));
        SmartDashboard.putNumber("Gyro Rate", Math.toRadians(gyro.getRate()));
        SmartDashboard.putNumber("Left Voltage", leftVoltage);
        SmartDashboard.putNumber("kV", kV);
    }
}
