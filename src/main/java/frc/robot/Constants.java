package frc.robot;

public class Constants {
    
    //Información Módulo Frente Izquierda (Usado en clase "Chasis")
    public static final int driveMotorIDfrenteIzquierda = 4;
    public static final int turnMotorIDfrenteIzquierda = 6;
    public static final int cancoderIDfrenteIzquierda = 1;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 0.2637 + 90;

    //Información Módulo Frente Derecha (Usado en clase "Chasis")
    public static final int driveMotorIDfrenteDerecha = 8;
    public static final int turnMotorIDfrenteDerecha = 5;
    public static final int cancoderIDfrenteDerecha = 2;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 68.1152 + 90;

    //Información Módulo Atrás Izquierda (Usado en clase "Chasis")
    public static final int driveMotorIDatrasIzquierda = 2;
    public static final int turnMotorIDatrasIzquierda = 7;
    public static final int cancoderIDatrasIzquierda = 3;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = 309.9023 + 90;

    //Información Módulo Atrás Derecha (Usado en clase "Chasis")
    public static final int driveMotorIDatrasDerecha = 3;
    public static final int turnMotorIDatrasDerecha = 1;
    public static final int cancoderIDatrasDerecha = 4;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 351.2988 + 90;

    //Valor de constante kP del PID [Asumiremos que tenemos que usar el mismo valor para los 4 módulos] (Usado en clase "Chasis")
    public static final double genericModulekP = 0.0048;
    public static final double genericModulekI = 0.0;
    public static final double genericModulekD = 0.0;

    public static final double standardTolerance = 0.03;



    public static final class ConstantesIO {
        public static final int idPuerto = 0;
        public static final int idPuerto_1 = 1; // Puerto detectado por el FRC Driver Station del control a usar
        
        /* Valores obtenidos experimentalmente através del FRC Driver Station */
        public static final int botonCuadrado = 1;
        public static final int botonCruz = 2;
        public static final int botonCirculo = 3;
        public static final int botonTriangulo = 4;
        public static final int bumperDerecho = 6;
        public static final int bumperIzquierdo = 5;
        public static final int gatilloIzquierdo = 7;
        public static final int gatilloDerecho = 8;
        public static final int flechaArriba = 0;
        public static final int flechaDerecha = 90;
        public static final int flechaAbajo = 180;
        public static final int flechaIzquierda = 270;

        public static final double kDeadband = 0.05;
    }
    
    public static final class DriveConstants {
        //Variable estableciendo la velocidad máxima de los módulos (Usado en clase "Chasis")
        public static final double maxSpeed = 0.4;
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //tunear segun requirimientos del robot
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.3;
    }

    public static final double driveRevsToMeters = 4 * Math.PI / (39.37 * 8.14)  * 1.3;
    public static final double driveRPS2MPS = driveRevsToMeters;



}
