package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase
{
    private SparkMax m_leader;
    private SparkMax m_follower1;
    private SparkMax m_follower2;
    private SparkMax m_follower3;
    private DigitalInput m_limitSwitch;
    private final SparkMaxAlternateEncoder m_encoder;
    private final SparkClosedLoopController m_liftController;
    
    private boolean inTolerance = false;
    public boolean disableElevator =true; //this is set to true to disable the elevator entirely for outreach (does not effect the wrist)

    public double kP_tune = Constants.ElevatorConstants.kP;
    public double PID_Tolerance_tune = Constants.ElevatorConstants.PID_TOLERANCE;
    public double e_speed_limit = Constants.ElevatorConstants.ELEVATOR_SPEED_MODIFIER;

    public ElevatorSubsystem()
    {
        m_leader = new SparkMax(Constants.ElevatorConstants.leaderPort, MotorType.kBrushed);
        m_follower1 = new SparkMax(Constants.ElevatorConstants.followerPort1, MotorType.kBrushed);
        m_follower2 = new SparkMax(Constants.ElevatorConstants.followerPort2, MotorType.kBrushed);
        m_follower3 = new SparkMax(Constants.ElevatorConstants.followerPort3, MotorType.kBrushed);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leaderConfig.inverted(false);
        leaderConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pidf(1.0, 0, 0, 0);
        AlternateEncoderConfig encoderConfig = new AlternateEncoderConfig();
        encoderConfig.setSparkMaxDataPortConfig();
        encoderConfig.inverted(true);
        leaderConfig.apply(encoderConfig);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(SparkMaxConfig.IdleMode.kBrake)
            // .disableFollowerMode();
            .follow(m_leader)
            .inverted(false);
        
        m_leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower1.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_follower3.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder = (SparkMaxAlternateEncoder)m_leader.getAlternateEncoder();
        m_liftController = m_leader.getClosedLoopController();

        //for tuning
        SmartDashboard.putNumber("Elevator kP", kP_tune);
        SmartDashboard.putNumber("Elevator PID Tolerance", PID_Tolerance_tune);

        m_limitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchPort);
        //SmartDashboard.putBoolean("Disable Elevator", disableElevator);
        //SmartDashboard.putBoolean("Check Elevator disable", disableElevator);

        // SmartDashboard.putNumber("Elevator Speed Limit (not for position control)", e_speed_limit);
        // SmartDashboard.putNumber("Elevator pidOutput", 9999);
        // SmartDashboard.putNumber("Elevator feedforward", 9999);
        // SmartDashboard.putNumber("Elevator speed", 9999);
        // SmartDashboard.putNumber("Elevator postion (from encoder)", m_leader.getEncoder().getPosition());

        m_leader.stopMotor();
        m_follower1.stopMotor();
        m_follower2.stopMotor();
        m_follower3.stopMotor();
        }

    public void periodic()
     {
        kP_tune = SmartDashboard.getNumber("Elevator kP", kP_tune);
        // pidController.setP(kP_tune);
        PID_Tolerance_tune = SmartDashboard.getNumber("Elevator PID Tolerance", PID_Tolerance_tune);
        // pidController.setTolerance(PID_Tolerance_tune);

        //disableElevator = SmartDashboard.getBoolean("Disable Elevator", disableElevator);
    }

    // manual control of the elevator
    /*public void moveElevator(double speed)
    {
        // SmartDashboard.putNumber("Elevator Position (from encoder)", m_leader.getEncoder().getPosition());
        if (disableElevator == false){ //safety feature for outreach
            if ((m_limitSwitch.get() && speed > 0) || m_leader.getEncoder().getPosition() < Constants.ElevatorConstants.MOTOR_TOP || m_leader.getEncoder().getPosition() > Constants.ElevatorConstants.MOTOR_BOTTOM)
            {
                speed = 0;
            }

            if(m_limitSwitch.get())
            {
                zeroElevator();
            }
            
            m_leader.set(speed);
        } else {
            m_leader.set(0);
        }
    } */

    public double getPosition()
    {
        return m_encoder.getPosition();
    }
    
    public RelativeEncoder getEncoder()
    {
        return m_encoder;
    }
    
    // do NOT setPosition outside of limit there is nothing stopping it for going out of bounds
    // public void setPostion(double goalPosition)
    // {
    //     // m_liftController.setReference(goalPosition, ControlType.kPosition);
    //     inTolerance = pidController.atSetpoint();

    //     // set the goal for it to -goalPosition, the encoder pose is - and the speed wants a + value
    //     pidController.setSetpoint(-goalPosition);

    //     // calculate the pid using feed forward and combine the values
    //     double pidOutput = pidController.calculate(getElevatorPostion(), goalPosition);
    //     double feedforwardOutput = feedforward.calculate(getElevatorPostion(), m_encoder.getVelocity());
    //     double speed = pidOutput + feedforwardOutput;

    //     // put the values on the smart dashboard
    //     // SmartDashboard.putNumber("Elevator pidOutput", pidOutput);
    //     // SmartDashboard.putNumber("Elevator feedforward", feedforwardOutput);
    //     // SmartDashboard.putNumber("Elevator speed", speed);
    //     // SmartDashboard.putNumber("Elevator postion (from encoder)", m_leader.getEncoder().getPosition());

    //     //ensure @pram speed is within -1 to 1
    //     speed = ( speed > 1) ? 1 :speed;
    //     speed = ( speed < -1) ? -1 : speed;

    //     // if the elevator is at its bottom, reset zero to compensate for encoder drifting through match
    //     if (m_limitSwitch.get() && speed > 0)
    //     {
    //         speed = 0;
    //         zeroElevator();
    //     }

    //     // set the motor speed
    //     if( disableElevator == false){ //safety feature for outreach
    //         //SmartDashboard.putBoolean("Check Elevator disable", disableElevator);
    //         m_leader.set(speed * Constants.ElevatorConstants.ELEVATOR_SPEED_MODIFIER);
    //     } else {
    //         m_leader.set(0);
    //     }
    // }

    // return the elevator encoder position
    public double getElevatorPostion()
    {
       return m_encoder.getPosition();
    }

    public void stopElevator()
    {
        m_leader.stopMotor();
    }

    public void zeroElevator()
    {
        if (disableElevator == false){ //safety feature for outreach
            m_encoder.setPosition(0);
        }
    }

    public boolean getInTolerance()
    {
        return inTolerance;
    }

    public void moveElevator(double speed) {
        m_leader.set(speed);
    }

    public void goToPosition(double rotations)
    {
        m_liftController.setReference(rotations, ControlType.kPosition);
    }
    
    public void dashboardStuff()
    {
        SmartDashboard.putNumber("lift Position", getElevatorPostion());
        SmartDashboard.putNumber("lift Applied Output", m_leader.getAppliedOutput());
        SmartDashboard.putNumber("lift Velocity", m_encoder.getVelocity());
    }
  }
