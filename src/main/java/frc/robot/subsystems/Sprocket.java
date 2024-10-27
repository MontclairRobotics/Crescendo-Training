package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Ports;
import frc.robot.RobotContainer;

public class Sprocket extends SubsystemBase {
    
    /* INSTANTIATES SPROCKET MOTORS */
    static CANSparkMax leftMotor = new CANSparkMax(Constants.Ports.LEFT_ANGLE_MOTOR, MotorType.kBrushless);
    static CANSparkMax rightMotor = new CANSparkMax(Constants.Ports.RIGHT_ANGLE_MOTOR, MotorType.kBrushless);
    
    /* INSTANCE VARIABLES */
    boolean canGoUp;
    boolean canGoDown;
    double inputForSprocket;
    boolean isAtSetPoint;
    double currentSetPoint;

    /* ABSOLUTE ENCODER */
    static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(Ports.SPROCKET_ABS_ENCODER); 

    /* PID CONTROLLER STUFF */
    static PIDController pidController = new PIDController(.8,0,0); 
    static boolean isUsingPID;
    boolean isSprocketSafe;
    
    /*
     * 
     * CONSTRUCTOR
     * INSTANTIATES SPROCKET
     * 
     */

    public Sprocket() {
        absoluteEncoder.setDistancePerRotation(360);
        Shuffleboard.getTab("Debug").addDouble("Sprocket Angle", getRawPositionSupplier());
        leftMotor.setInverted(true);
        rightMotor.setInverted(true);
        pidController.setTolerance(1);
    }

    /*
     * 
     * METHODS
     * 
     */

    /* GETS THE ENCODER ANGLE (THE ACTUAL VALUE) */
    public static double getRawPosition(){
        return absoluteEncoder.getDistance() + ArmConstants.SPROCKET_OFFSET; 
    }

    /* GETS THE ENCODER ANGLE, FOR LOGGING PURPOSES */
    public DoubleSupplier getRawPositionSupplier(){
        return () -> getRawPosition();
    }
    
    /**
     * SETS ANGLE USING PID
     * 
     * @param angle is in rotation 2d
     */

    public void setAngle(Rotation2d angle){
        if(angle.getDegrees() > 63 || angle.getDegrees() < 26) stop();
        else pidController.setSetpoint(angle.getDegrees());

        /* THIS IS TO CHECK IF SPROCKET IS AT ANGLE */
        currentSetPoint = angle.getDegrees();

        isUsingPID = true;
       // System.out.println("isUsingPid is being set to true");
    }

    /* STOPS THE SPROCKET MOTORS */
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }

    /* SETS BRAKE MODE TO BOTH SPROCKET MOTORS */
    public void setBrakeMode(){
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }
    
    /* SETS BOTH MOTORS TO COAST MODE */
    public void setCoastMode (){
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.setIdleMode(IdleMode.kCoast);
    }
   
    /* SPROCKET DEFAULT INPUT METHOD USING THE JOYSTICK */
    public void sprocketDefault(){
        inputForSprocket = RobotContainer.operatorController.getLeftY();
        inputForSprocket = -1 * inputForSprocket * inputForSprocket * inputForSprocket;
    }
   
    /* BOOLEAN SUPPLIER FOR CHECKING THAT THE SPROCKET IS AT THE ANGLE IT IS SET TO */
    public BooleanSupplier isAtAngleSupplier(){
        return () -> isAtSetPoint;
    }

    public boolean isAtAngle(){
        return isAtSetPoint;
    }

    /*
     * 
     * 
     * PERIODIC METHOD
     * CALLED ONCE EVERY SCHEDULED RUN
     * 
     * 
     */

    public void periodic() {

        //sets the is at setpoint method
        isAtSetPoint = Math.abs(currentSetPoint - getRawPosition()) < ArmConstants.SPROCKET_ANGLE_DEADBAND;

        //safety to protect motors
         if(Math.abs(inputForSprocket)<.04){
            inputForSprocket = 0;
        } else isUsingPID = false;

            if(isUsingPID && (!canGoUp && pidController.getSetpoint() > 63 || !canGoDown && pidController.getSetpoint() < 26)) {
                leftMotor.set(0);
                rightMotor.set(0);
            } else if (isUsingPID) {
                leftMotor.setVoltage(pidController.calculate(getRawPosition()));
                rightMotor.setVoltage(pidController.calculate(getRawPosition()));
            } else if((!canGoUp && inputForSprocket > 0 ) || (!canGoDown && inputForSprocket < 0)) {
                leftMotor.set(0);
                rightMotor.set(0);
            } else if (Math.abs(inputForSprocket) > 0){
                isUsingPID = false;
                leftMotor.set(inputForSprocket);
                rightMotor.set(inputForSprocket);
            } else {
                leftMotor.set(0);
                rightMotor.set(0);
            }

        //safety booleans
        canGoUp = getRawPosition() < (ArmConstants.ENCODER_MAX_ANGLE - ArmConstants.SPROCKET_ANGLE_DEADBAND);
        canGoDown = getRawPosition() > (ArmConstants.ENCODER_MIN_ANGLE - ArmConstants.SPROCKET_ANGLE_DEADBAND);
        isSprocketSafe = canGoUp && canGoDown;
    }
}
