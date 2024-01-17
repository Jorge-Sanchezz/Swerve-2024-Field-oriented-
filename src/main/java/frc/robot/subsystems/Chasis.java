package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chasis extends SubsystemBase {
        //Valores puntuales y fijos que tendrá cada módulo como el ID que tienen sus motores, su valor de PID, etc. Los datos estan almacenados en la
        //clase "Constants" y se estan accediendo a partir de la nomenclatura Constants.'nombre de variable'
        private Module frenteIzquierda = new Module(Constants.driveMotorIDfrenteIzquierda, 
                                    Constants.turnMotorIDfrenteIzquierda, 
                                    Constants.cancoderIDfrenteIzquierda, 
                                    Constants.genericModulekP, 
                                    Constants.genericModulekI, 
                                    Constants.genericModulekD, 
                                    Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

        private Module frenteDerecha = new Module(Constants.driveMotorIDfrenteDerecha, 
                                Constants.turnMotorIDfrenteDerecha, 
                                Constants.cancoderIDfrenteDerecha, 
                                Constants.genericModulekP, 
                                Constants.genericModulekI, 
                                Constants.genericModulekD,
                                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET);

        private Module atrasIzquierda = new Module(Constants.driveMotorIDatrasIzquierda, 
                                    Constants.turnMotorIDatrasIzquierda, 
                                    Constants.cancoderIDatrasIzquierda, 
                                    Constants.genericModulekP, 
                                    Constants.genericModulekI, 
                                    Constants.genericModulekD,
                                    Constants.BACK_LEFT_MODULE_STEER_OFFSET);

        private Module atrasDerecha = new Module(Constants.driveMotorIDatrasDerecha, 
                                Constants.turnMotorIDatrasDerecha, 
                                Constants.cancoderIDatrasDerecha, 
                                Constants.genericModulekP, 
                                Constants.genericModulekI, 
                                Constants.genericModulekD,
                                Constants.BACK_RIGHT_MODULE_STEER_OFFSET);
    
    

    //Valores de "x" y "y" que representan la ubicación de cada módulo en el chasís
    Translation2d frenteIzqTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d frenteDerTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(11.5)); //Units in Meters
    Translation2d atrasIzqTranslation = new Translation2d(Math.toRadians(-11.5), Math.toRadians(-11.5)); //Units in Meters
    Translation2d atrasDerTranslation = new Translation2d(Math.toRadians(11.5), Math.toRadians(-11.5)); //Units in Meters

    //Declarar giroscopio Pigeon 2
    final Pigeon2 gyro = new Pigeon2(0, "Drivetrain");

    final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frenteIzqTranslation, frenteDerTranslation, atrasIzqTranslation, atrasDerTranslation);

    SwerveModulePosition[] positions = {frenteIzquierda.getPosition(), frenteDerecha.getPosition(), 
        atrasIzquierda.getPosition(), atrasDerecha.getPosition()};

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    /*
    //Método que sería usado si no quisieramos que el robot tuviera orientación a cancha (esta manera no implementa el giroscopio) 
    
    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    
        setModuleStates(states);
    }
   */
    
    
    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){
        ChassisSpeeds chassisSpeedsFieldOriented = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());    
    
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeedsFieldOriented);
        
        //Método que da un límite a todos los módulos de manera proporcional para mantener el comportamiento deseado al mover el robot
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.DriveConstants.maxSpeed);

        setModuleStates(states);
    }
    
    

    public void setModuleStates(SwerveModuleState[] states){
        /*Importante asignar los valores del array en este orden porque así fue como se
        declaró cada estado por separado y tienen que ser asignados al indicado*/
        frenteIzquierda.setDesiredState(states[1], "Frente Izquierda");
        frenteDerecha.setDesiredState(states[0], "Frente Derecha");
        atrasIzquierda.setDesiredState(states[3], "Atras Izquierda");
        atrasDerecha.setDesiredState(states[2], "Atras Derecha");
    }

    public double getAngle(){
        //Funcion para pedirle ap Pigeon el angulo que esta midiendo
        return (this.gyro.getAngle())%360;
    }
/*
    public ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        moduleStates[1] = frenteIzquierda.getSwerveState();
        moduleStates[0] = frenteDerecha.getSwerveState();
        moduleStates[3] = atrasIzquierda.getSwerveState();
        moduleStates[2] = atrasDerecha.getSwerveState();
        return kinematics.toChassisSpeeds(moduleStates);
    }

    
     * 
     * 
     * 
     */
public ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds(0.1, 0.1, 0.1);
}

    public ChassisSpeeds getRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds){
        return new ChassisSpeeds(0.1, 0.1, 0.1);
    }

    public ChassisSpeeds getFieldOrienteSpeeds(){
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation2d());
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(Math.toRadians(getAngle()));
    }

    public Pose2d getPose2d(){
        //return poseEstimator.getEstimatedPosition();
        return odometry.getPoseMeters();
    }

    public void setOdoPose(Pose2d pose){
        positions[1] = frenteIzquierda.getPosition();
        positions[0] = frenteDerecha.getPosition();
        positions[3] = atrasIzquierda.getPosition();
        positions[2] = atrasDerecha.getPosition();

        odometry.resetPosition(getRotation2d(), positions, pose);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Chasis(){
        AutoBuilder.configureHolonomic(
            this::getPose2d, 
            this::setOdoPose, 
            this::getChassisSpeeds, 
            this::getRelativeChassisSpeeds, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(0, 0, 0), 
                new PIDConstants(0, 0, 0), 
                4.5,
                0.4,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                
                }
                return false;
            },
            this);
    }

    @Override
    //El periodic funciona para ver cosas mínimas dentro del subsistema (Funciona aún cuando esta desabilitado)
    public void periodic() {
        //Herramienta de first para imprimir valorres que es mucho más eficaz en términos de velocidad a comparacion al System.out.println
        //En este caso estamos pidiendo el ángulo que esta midiendo el giroscopio
        SmartDashboard.putNumber("gyro angle",getAngle());
        SmartDashboard.putNumber("pose x", getPose2d().getX());
        SmartDashboard.putNumber("pose y", getPose2d().getY());
        SmartDashboard.putNumber("gyro angle", getAngle());
        SmartDashboard.putNumber("frenteIzquierda angle", this.frenteIzquierda.getAngle().getDegrees());
        SmartDashboard.putNumber("frenteDerecha angle", this.frenteDerecha.getAngle().getDegrees());
        SmartDashboard.putNumber("atrasIzquierda angle", this.atrasIzquierda.getAngle().getDegrees());
        SmartDashboard.putNumber("atrasDerecha angle", this.atrasDerecha.getAngle().getDegrees());

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        double found = table.getEntry("tv").getDouble(0);

        double[] poseNums = new double[6];
        poseNums = table.getEntry("botpose_wpiblue").getDoubleArray(poseNums);

        Pose2d visionMeasurement = new Pose2d(poseNums[0], poseNums[1], getRotation2d());

        SmartDashboard.putNumber("gyro angle",getAngle());
        SmartDashboard.putNumber("pose x", getPose2d().getX());

        positions[0] = frenteIzquierda.getPosition();
        positions[1] = frenteDerecha.getPosition();
        positions[2] = atrasIzquierda.getPosition();
        positions[3] = atrasDerecha.getPosition();
        odometry.update(getRotation2d(), positions);

        poseEstimator.update(getRotation2d(), positions);

        if(found == 1){
            poseEstimator.addVisionMeasurement(visionMeasurement, Timer.getFPGATimestamp(), null);
        }
        

    }
}
