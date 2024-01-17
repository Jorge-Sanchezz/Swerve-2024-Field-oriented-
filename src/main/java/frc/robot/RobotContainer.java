// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autonomo;
import frc.robot.commands.Manejo;
import frc.robot.subsystems.Chasis;

//El container construye el chasis y el módulo directamente (accede a esas clases estableciendo unión al código del robot)
public class RobotContainer {
  private static final Chasis chasis = new Chasis();
  private static final PS4Controller driveControl = new PS4Controller(0);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    //Para que la información de los controles se actualice constantemente, el método get de la clase "Manejo" pedirá la información 
    //a esta clase, para así en vez de dejar un valor fijo, asignar valores requeridos al robot constantemente.
    /* */
    chasis.setDefaultCommand(
      new Manejo(
        chasis,
        () -> (-driveControl.getRawAxis(0)),
        () -> (-driveControl.getRawAxis(1)),
        () -> (driveControl.getRawAxis(2))
      )
    );
    
    configureBindings();
    

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }
  

  private void configureBindings() {
    new JoystickButton(driveControl, Constants.ConstantesIO.botonTriangulo).whileTrue(new RunCommand(chasis::zeroHeading));
  }

  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();


    /*
    PathPlannerPath path = PathPlannerPath.fromPathFile("Copy of Example Path");

    return AutoBuilder.followPath(path);
    //return new Autonomo(chasis);
    */
  }
}
