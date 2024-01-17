package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chasis;

public class Autonomo extends Command{
     Chasis chasis;
     double xObjective, yObjective, zObjective;
     int estadoAutonomo = 0;

    PIDController drivePID = new PIDController(0.3,0,0);
    PIDController rotationPID = new PIDController(0.3,0,0);

     public Autonomo(Chasis chasis){
        this.chasis = chasis;
     }

      @Override
    public void initialize() {
        yObjective = 1;
        xObjective = chasis.getPose2d().getX();
        zObjective = 180;
    }

    @Override
    public void execute(){
        if(estadoAutonomo == 0){
            yObjective = 50;
            xObjective = chasis.getPose2d().getX();
            zObjective = 180;
        } else if (estadoAutonomo == 1){
            yObjective = 1;
            xObjective = 1;
            zObjective = 270;
        }



        double xSpeed = drivePID.calculate(chasis.getPose2d().getX(), xObjective);
        double ySpeed = drivePID.calculate(chasis.getPose2d().getY(), yObjective);
        double zSpeed = rotationPID.calculate(chasis.getPose2d().getRotation().getDegrees(), zObjective);
                
        System.out.println("getY " + chasis.getPose2d().getY());
        System.out.println("yObjective " + yObjective);
        System.out.println("ySpeed "+ ySpeed);
        System.out.println("================");

        if(false && Math.abs(xSpeed) < 0.1 && Math.abs(ySpeed) < 0.1 && Math.abs(zSpeed) < 0.1){
            estadoAutonomo++;
        }

        chasis.setFieldOrientedSpeed(xSpeed, ySpeed, zSpeed);

    }
}

/*
package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.Constants;
import frc.robot.subsystems.Chasis;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class Autonomo extends Command{

    private Chasis chasis;
    Timer autonomoTimer;
    
    
    public Autonomo(Chasis chasis){
        this.chasis = chasis;
        autonomoTimer = new Timer();
        autonomoTimer.start();

        addRequirements(chasis);
    }

    private static boolean done;
    private static boolean flag;
    



    @Override
    public void initialize() {
  
      done = false;
      flag = false;

      chasis.zeroHeading();
      System.out.println("Autos moda y rock&roll command started");
    }

    

    @Override
    public void execute(){
 
      if(!done){
      flag = false;
      while(!flag){
        if(autonomoTimer.get() > 3){
            flag = true;
        }
        chasis.setFieldOrientedSpeed(0,-0.6, 0);
        if(chasis.getPose2d().getX() < -2.4){
          flag = true;
        }
      }
      }
      done = true;
      
      this.chasis.setFieldOrientedSpeed(0, 0, 0);
        
        
      

    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Autos moda y rock&roll  command ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    return false;
  }



}
*/