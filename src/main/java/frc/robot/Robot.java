/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */

public class Robot extends TimedRobot {
    // Constantes
    final double VEL_CORREA = 0.40;
    final double VEL_SHOOTER = 0.60;
    final double VEL_RECOGEDOR = 0.40;

    // ---------------- MOTORES ------------ //
    private TalonSRX leftTalon = new TalonSRX(1);
    private TalonSRX rightTalon = new TalonSRX(7);
    // private TalonSRX RecogedorTalon2 = new TalonSRX(5);
    // private TalonSRX RecogedorTalon1 = new TalonSRX(6);
    private TalonSRX correaTalon1 = new TalonSRX(5);
    private TalonSRX correaTalon2 = new TalonSRX(6);
    private TalonSRX leftShooterTalon = new TalonSRX(4);
    private TalonSRX rightShooterTalon = new TalonSRX(8);
    // ---------------- FIN MOTORES ------------ //

    // ---------------- CONTROLES ------------ //
    private XboxController ControlDriver = new XboxController(2);

    // ---------------- FIN CONTROLES ------------ //

    // ----------------- VARIABLES ---------------//
    private boolean recogedorToogle = false;
    private boolean CorreaToggle = false;
    private boolean shooterToggle = false;

    private DifferentialDrive m_robotDrive;
    private Joystick m_stick;

    private double startTime;

    // ------- Variables para el Shooter ----
    private double angulo_shooter = 40; // en Grados para entender
    private double radianes_shooter = angulo_shooter / 180 * 01;

    @Override
    public void robotInit() {
        // Invert the left side motors.
        // You may need to change or remove this to match your robot.
        // frontLeft.setInverted(true);
        // rearLeft.setInverted(true);
        // DifferentialDrive m_robotDrive = new DifferentialDrive(leftTalon,
        // rightTalon);
        m_stick = new Joystick(2);
    }

    @Override
    public void autonomousInit() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void autonomousPeriodic() {
        double time = Timer.getFPGATimestamp();
        if (time - startTime < 3) {
            avanzar_por_velocidad(0.6);
        } else {
            avanzar_por_velocidad(0.0);
        }
    }

    @Override
    public void teleopPeriodic() {
        acelerar_robot(ControlDriver.getY(Hand.kLeft) * -1, ControlDriver.getY(Hand.kRight));

        // recogedor toggle ver.
        if (ControlDriver.getYButtonPressed()) {
            correa_bajando_boton(); // Regurgitar
        }

        if (ControlDriver.getAButtonPressed()) {
            correa_subiendo_boton(); // Recoger
        }

        // SHOOTER BUTTON
        if (ControlDriver.getXButtonPressed()) {
            shooter_encendido_boton();
        }

    }
    // ------------------------------------------

    /* ------ METODOS PARA EL RECOGEDOR --- */

    private void acelerar_recogedor(double vel) {
        // recogedorTalon1.set(ControlMode.PercentOutput, vel);
        // recogedorTalon2.set(ControlMode.PercentOutput, vel);

    }

    private void detener_recogedor() {
        acelerar_recogedor(0.0);
    }

    private void recogedor_entrando() {
        acelerar_recogedor(VEL_RECOGEDOR);
    }

    private void recogedor_saliendo() {
        acelerar_recogedor(-1 * VEL_RECOGEDOR);
    }

    private void recogedor_entrando_boton() {
        if (recogedorToogle == false) { // ya esta apagado
            recogedorToogle = true; // corriendo
            recogedor_entrando();
        } else {
            detener_recogedor();
            recogedorToogle = false;
        }
    }

    private void recogedor_saliendo_boton() {
        if (recogedorToogle == false) { // ya esta apagado
            recogedorToogle = true; // corriendo
            recogedor_saliendo();
        } else {
            detener_recogedor();
            recogedorToogle = false;
        }
    }

    /* ------ METODOS PARA LA CORREA --- */

    private void correa_subiendo() {
        acelerar_Correa(VEL_CORREA);
    }

    private void correa_bajando() {
        acelerar_Correa(-1.0 * VEL_CORREA);
    }

    private void detener_correa() {
        acelerar_Correa(0.0);
    }

    private void acelerar_Correa(double vel) {
        correaTalon1.set(ControlMode.PercentOutput, vel);
        correaTalon2.set(ControlMode.PercentOutput, vel);

    }

    private void correa_bajando_boton() {
        if (CorreaToggle == false) { // ya esta apagado
            CorreaToggle = true; // corriendo
            correa_bajando();
        } else {
            detener_correa();
            CorreaToggle = false;
        }
    }

    private void correa_subiendo_boton() {
        if (CorreaToggle == true) {
            CorreaToggle = false;
        } else if (CorreaToggle == false) {
            CorreaToggle = true;
        }
        if (CorreaToggle == true) {
            correa_subiendo();
        }
        if (CorreaToggle == false) {
            detener_correa();

        }

    }

    /* ------ METODOS PARA EL SHOOTER --- */
    private void detener_shooter() {
        acelerar_shooter(0.0);
    }

    private void acelerar_shooter(double vel) {
        leftShooterTalon.set(ControlMode.PercentOutput, -vel);
        rightShooterTalon.set(ControlMode.PercentOutput, -vel);
    }

    private void shooter_encendido_boton() {

        if (shooterToggle == true) {
            shooterToggle = false;
        } else if (shooterToggle == false) {
            shooterToggle = true;
        }
        if (shooterToggle == true) {
            acelerar_shooter(VEL_SHOOTER);

        }
        if (shooterToggle == false) {
            detener_shooter();

        }
    }

    private void avanzar_por_velocidad(double vel) {
        acelerar_robot(-1.0 * vel, vel);
    }

    private void acelerar_robot(double vel_left, double vel_right) {
        leftTalon.set(ControlMode.PercentOutput, vel_left);
        rightTalon.set(ControlMode.PercentOutput, vel_right);
    }

}
