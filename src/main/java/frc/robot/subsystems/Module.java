package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Module {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final CANcoder turnEncoder;
    
    private final PIDController turnPID;

    private final double offset;



    public Module(int driveMotorID, int turnMotorID, int CANcoderID, double kP, double kI, double kD , double turnEncoderOffset) {
        this.offset = turnEncoderOffset;

        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        this.driveMotor.restoreFactoryDefaults();
        this.turnMotor.restoreFactoryDefaults(); 

        this.turnEncoder = new CANcoder(CANcoderID, "Drivetrain");

        this.turnPID = new PIDController(kP, kI, kD);

        //this.turnPID.setTolerance(0.1);

        this.turnPID.enableContinuousInput(-180, 180);

    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePosition(), getAngle());
      }   

    public void setDesiredState(SwerveModuleState desiredState, String moduleName){
        desiredState =  SwerveModuleState.optimize(desiredState, getAngle());

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.05){
            stop();
            return;
            }
        setSpeed(desiredState, moduleName);
        setAngle(desiredState, moduleName);

    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void setSpeed(SwerveModuleState desiredState, String moduleName){
        driveMotor.set(desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("Drive motor output " + moduleName, desiredState.speedMetersPerSecond);
    }

    public void setAngle(SwerveModuleState desiredState, String moduleName){
        double realAngle =  getAngle().getDegrees() /*+ this.offset*/;
        double desiredAngle = desiredState.angle.getDegrees();
        double PIDvalue = turnPID.calculate(realAngle, desiredAngle);
        turnMotor.set(PIDvalue);
        SmartDashboard.putNumber("Real angle module " + moduleName, realAngle);
        SmartDashboard.putNumber("Desired angle module " + moduleName, desiredAngle);
        SmartDashboard.putNumber("Turn motor output " + moduleName, PIDvalue);
    }
    
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition().getValue()*-360 + this.offset);
    }

    public SwerveModuleState getSwerveState(){
        return new SwerveModuleState(getDriveVelocity(), getAngle());
     }

     public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity() * Constants.driveRPS2MPS;
    }

    public double getDrivePosition(){
        double position;

        position = driveMotor.getEncoder().getPosition();

        position *= Constants.driveRevsToMeters;
        
        return position;
    }
}
