/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.lang.Math;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//para poder utilizar el imelight a traves del networktable

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive
 * class.
 */

public class Robot extends TimedRobot {

    // ID de Talons en Robot Nuevo
    final int DRIVE_LF = 13;
    final int DRIVE_LB = 14;
    final int DRIVE_RF = 3;
    final int DRIVE_RB = 4;

    final int CORREA_F = 1;
    final int CORREA_B = 2;
    final int RECOGEDOR_F = 10;
    final int RECOGEDOR_B = 11;

    final int SHOOTER_L = 15;
    final int SHOOTER_R = 6;
    final int DOSIFICADOR = 5 ;

    final int LIFTER_L = 16;
    final int LIFTER_R = 7;

    final int RULETA = 12;

    // Constantes
    final double VEL_CORREA = 0.25;
    final double VEL_SHOOTER = 0.80;
    final double VEL_RECOGEDOR = 0.25;
    final double VEL_DOSIFICADOR= 0.6;

    private BajandoPelotas bajandoPelotas;

    // ---------------- MOTORES  ------------ //
    private WPI_TalonSRX leftFrontDrive = new WPI_TalonSRX(DRIVE_LF);
    private WPI_TalonSRX leftBackDrive = new WPI_TalonSRX(DRIVE_LB);
    private WPI_TalonSRX frontRecogedorMotor = new WPI_TalonSRX(RECOGEDOR_F);
    private WPI_TalonSRX rearRecogedorMotor = new WPI_TalonSRX(RECOGEDOR_B);
    private WPI_TalonSRX dosificadorMotor = new WPI_TalonSRX(DOSIFICADOR);
    private WPI_TalonSRX leftShooterMotor = new WPI_TalonSRX(SHOOTER_L);



    private WPI_TalonSRX rightFrontDrive = new WPI_TalonSRX(DRIVE_RF);
    private WPI_TalonSRX rightBackDrive = new WPI_TalonSRX(DRIVE_RB);
    private WPI_TalonSRX correaFrontMotor = new WPI_TalonSRX(CORREA_F);
    private WPI_TalonSRX correaBackMotor = new WPI_TalonSRX(CORREA_B);
    private WPI_TalonSRX rightShooterMotor = new WPI_TalonSRX(SHOOTER_R);
    // ---------------- FIN MOTORES  ------------ //


    // ---------------- Limelight ------------ //
    NetworkTable table;
    NetworkTableEntry txLimelight;
    NetworkTableEntry tyLimelight;
    NetworkTableEntry taLimelight;
    // ---------------- Limelight ------------ //

    // ---------------- CONTROLES ------------ //
    private XboxController controlDriver = new XboxController(2);
    private XboxController assistantDriver = new XboxController(0);

    // ---------------- FIN CONTROLES ------------ //

    // ---------------- NEUMATICA --------------- //

    private Compressor mainCompressor = new Compressor(0);
    private DoubleSolenoid shooterSolenoid = new DoubleSolenoid(2, 3);
    boolean enabled;
    boolean pressureSwitch;
    double current;
    // ------------- FIN DE NEUMATICA ----------- //

    // ---------------- SENSORES ------------ //
    private DigitalInput sensorAbajo = new  DigitalInput(9);
    private DigitalInput sensorMedio = new  DigitalInput(8);
    private DigitalInput sensorArriba = new  DigitalInput(7);

    // ---------------- FIN SENSORES ------------ //

    // ----------------- VARIABLES ---------------//

    /* --- BANDERAS PARA ESTADOS --- */
    private boolean subiendoActivo = false;
    private boolean bajandoActivo = false;
    private boolean shooterActivo = false;
    private boolean alineandoDistanciaActivo = false;
    private boolean alineandoActivo = false;
    private boolean dispararDosificadorActivo = false; //indica si activado
    private boolean dosificadorActivo = false; //indica si activado
    private boolean ascensorActivo = false;
    private boolean descargaActivo = false;
    private boolean recogedorSaliendoActivo = false;

    /* --- INDICADORES  --- */

    // Indica si un estado esta bloqueando el uso del drivetrain (por el Driver)
    private boolean drivetrainBloqueado = false;

    // Indica si se esta manejando (el Driver) con los motores invertidos
    // TODO: Asegurarse que todo lo demas que mueve el vehiculo lo revise
    private boolean motoresInvertidos = false;

    // Indica si el shooter esta inclinado a 45 grados u horizontal.
    private boolean shooterInclinado = false;

    // private DifferentialDrive m_robotDrive;
    // private Joystick m_stick;

    private double startTime;

    // ---- Variables para Limelight ----- //
    private double Val_Dx = 0.0;
    private double Val_P = 0.0;
    private double tx_mem = 0.0;
    private double ty_mem = 0.0;
    private double align_mem = 0.0;
    private double ty_distance = 0.0;
    private double Kp = 0.0;
    private double Ki = 0.0;
    private double error_min = 15.0;

    private double currentDistance = 0.0;
    private double goalDistance = 0.0; // pulgadas
    private double distanceError = 0.0;
    private double kp_dist = 0.0;
    private double rangeAdjustment = 0.0;

    private double combinacionVelocidadPos = 0.0;
    private double combinacionVelocidadNeg = 0.0;

    // Para el cargando_pelotas

    private boolean ascensorCargado = false;
    private boolean pelotaDisponible = false;
    private boolean subiendoPelota = false;
    private boolean pelotaS2 = false;
    private boolean pelotaLista = false;
    private int contadorDescargar = 0;


    // ------- Variables para el Shooter ----
    private double angulo_shooter = 40; // en Grados para entender
    private double radianes_shooter = angulo_shooter / 180 * 01;
    private int tiempo_dosificador=10;
    private int contador_dosificador = 0;

    //Variables para prueba de shooter
    private int contadorPrueba=0;
    private boolean verificadorPrueba= false;

    @Override
    public void robotInit() {

        bajandoPelotas = new BajandoPelotas(this);
        // m_stick = new Joystick(2);
        iniciar_neumatica();

        table = NetworkTableInstance.getDefault().getTable("limelight");
        txLimelight = table.getEntry("tx");
        tyLimelight = table.getEntry("ty");

        alineandoActivo = false;
        alineandoDistanciaActivo = false;
        motoresInvertidos = false;
        Kp = 0.05;
        Ki = 0.002;
        // taLimelight = table.getEntry("ta");
    }

    @Override
    public void robotPeriodic() {
        //detector();


        /* --- Mostrar Estados en Dashboard --- */
        // De Torreta
        SmartDashboard.putBoolean("ASCENSOR", ascensorActivo);
        SmartDashboard.putBoolean("DESCARGA", descargaActivo);
        SmartDashboard.putBoolean("Subiendo", subiendoActivo);
        SmartDashboard.putBoolean("Bajando", bajandoPelotas.estaActivo());

        // De Disparador
        SmartDashboard.putBoolean("DOSIFICADOR", dosificadorActivo);
        SmartDashboard.putBoolean("Shooter", shooterActivo);

        // De Movimiento
        SmartDashboard.putBoolean("Alineando", alineandoActivo);
      
        /* --- Mostrar Indicadores --- */
        // De Torreta
        SmartDashboard.putBoolean("Cargado", ascensorCargado);
        SmartDashboard.putBoolean("Subiendo Pelota", subiendoPelota);
        SmartDashboard.putBoolean("Disponible", pelotaDisponible);
        SmartDashboard.putBoolean("Pelota 1", !sensorAbajo.get());
        SmartDashboard.putBoolean("Pelota 2", !sensorMedio.get());
        SmartDashboard.putBoolean("Pelota 3", !sensorArriba.get());

        // De Disparador

        // De Movimiento
        SmartDashboard.putBoolean("M Invertidos", motoresInvertidos);
        SmartDashboard.putBoolean("Drivetrain Bloqueado", drivetrainBloqueado);



    }

    @Override
    public void disabledInit() {

        reiniciar_estados();

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
    public void teleopInit() {
        // Otras variables
        subiendoPelota = false;
    }

    /*
    TeleopPeriodic
    Ejecuta nuestro programa:
     Primero: Leer sensores, botones, etc.
     Segundo: Ejecutar estados o tareas
     Tercero: Leer el joystick
    */
    @Override
    public void teleopPeriodic() {


        // ------------- LEER SENSORES -------- //
        pelotaLista = !sensorArriba.get();
        ascensorCargado = pelotaLista && !sensorMedio.get();
        pelotaDisponible = !sensorAbajo.get(); //Pelota pelotaDisponible para subir

        // ------------ LEER BOTONES ------------//
        // Del Control Princpipal (driver)
        invertir_motores_boton_Y(); // no finciona
        shooter_encendido_boton_X(); // no funciona
        cambiar_angulo_shooter_boton_A();
        recogedor_saliendo_boton_y();
        detener_estados_boton_back();
        descarga_boton_L1();
        ascensor_boton_R1();
        disparar_dosificador_boton_B();

        // Del Control Secundario (Asistente)
        subiendo_boton_2Y();
        bajando_boton_2A();
        correr_dosificador_boton_2B();

        // ----------- REVISAR TAREAS Y ESTADOS -----------

        // Las tareas generalmente serian exclusivas (solo una a la vez)

        /* Maquina para ejecutar tareas */



        // Hay estados que son exclusivos, otros que no.
        // Para los exclusivos: if .... else if
        // Para los demas: if .... if ..

        /* Maquina para ejecutar estados exclusivos */
        if(descargaActivo){
            descargando_pelotas();
        }
        else if(ascensorActivo){
            cargando_pelotas();
        }
        else if(subiendoActivo){

            subiendo_pelotas();
        }
        else if(bajandoPelotas.estaActivo()){
            // bajando_pelotas();
            bajandoPelotas.ejecutar();
        }

        /* Maquina para ejecuta estados no exclusivos */
        if(dosificadorActivo){
            dosificador_encendido();
        } else if(dispararDosificadorActivo){
            dosificador_disparando();
        }

        if(shooterActivo){
            shooter_encendido();
        }
        if(alineandoActivo){
            robot_alineandose();
        }



        mover_con_joysticks();
    }
    // ------------------------------------------

    /* ------ ESTADOS DEL ROBOT ------------- */
    // TODO: ponerlas en el mismo orden en que aparecen los estados en
    // TeleopPeriodic
    
    /* --- ESTADOS DE LA TORRETA --- */
    private void bajando_pelotas(){
        correa_bajando();
        recogedor_saliendo();
    }

    private void subiendo_pelotas(){
        correa_subiendo();
        recogedor_entrando();
    }

    private void recogedor_entrando() {
        acelerar_recogedor(-1*VEL_RECOGEDOR);
    }

    private void recogedor_saliendo() {
        acelerar_recogedor(VEL_RECOGEDOR);
    }

    private void correa_subiendo() {
        acelerar_Correa(VEL_CORREA);
    }

    private void correa_bajando() {
        acelerar_Correa(-1.0 * VEL_CORREA);
    }

    /* --- ESTADOS DEL DISPARADOR --- */
    private void dosificador_encendido(){
        acelerar_dosificador(0.3*VEL_DOSIFICADOR);
    }

    private void shooter_encendido() {
        acelerar_shooter(VEL_SHOOTER);
    }


    /* --- ESTADOS DE MOVIMIENTO --- */
    private void robot_detenido() {
        // acelerar_robot(controlDriver.getY(Hand.kLeft) * -1,
        // controlDriver.getY(Hand.kRight));
    }

    private void robot_alineandose() {
        alinear_con_camara();
    }

    /* ----------- METODOS PARA CAMBIAR DE ESTADOS ------------- */

    private void reiniciar_estados() {
        // Estados
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        alineandoActivo = false;

        alineandoDistanciaActivo = false;
        shooterInclinado = false;
        dispararDosificadorActivo = false;
        dosificadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;

        drivetrainBloqueado = false;

    }

    private void activar_robot_detenido() {

        shooterActivo = false;
        subiendoActivo = false;
        alineandoActivo = false;
        // alineandoDistanciaActivo = false;
        bajandoActivo = false;
        drivetrainBloqueado = false;
    }

    private void activar_shooter_encendido() {
        shooterActivo = true;
    }

    private void activar_recogedor_saliendo() {
        subiendoActivo = true;

        alineandoActivo = false;
        // alineandoDistanciaActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
    }

    private void activar_subiendo(){
        subiendoActivo = true;

        bajandoActivo = false;
        dispararDosificadorActivo = false;
        dosificadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_bajando(){
        bajandoActivo = true;

        subiendoActivo = false;
        dispararDosificadorActivo = false;
        dosificadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_robot_alineandose() {
        alineandoActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        motoresInvertidos = false;
        drivetrainBloqueado = false;

        alineandoDistanciaActivo = false;
        dispararDosificadorActivo = false;
        dosificadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_disparar_dosificador() {
        // alineandoDistanciaActivo = false;
        dispararDosificadorActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        alineandoActivo = false;
        dosificadorActivo = false;
        ascensorActivo = false;
        descargaActivo = false;
}

    private void activar_descarga(){
        descargaActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        dispararDosificadorActivo = false;
        dosificadorActivo = false;
        ascensorActivo = false;
    }

    private void activar_ascensor(){
        ascensorActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        dispararDosificadorActivo = false;
        dosificadorActivo = false;
        descargaActivo = false;
    }

    private void activar_dosificador(){
        dosificadorActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;

        dispararDosificadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_invertir(){
        motoresInvertidos = true;
        shooterActivo = false;
        alineandoActivo = false;

        alineandoDistanciaActivo = false;
    }


    /* -------- ESTADOS -------- */
    private void cargando_pelotas(){
        //SUBIENDO: Si llega una pelota, subirla hasta el proximo nivel (S2)
        //CARGADO: Si llega una al tercer nivel, y el segundo no esta vacio, detengo el recogedor

        // CARGADO: Hay una el el tercero, y una en el segundo (ya no subir mas pelotas) (puedo recoger, pero no subo)
        // SUBIENDO: Llega una a S1 y no hay en S2
        if (!ascensorCargado){
            if(pelotaDisponible ){
                subiendoPelota = true;
                if (!sensorMedio.get()){
                    pelotaS2 = true;
                } else {
                    pelotaS2 = false;
                }
            }
        }

        if (ascensorCargado && pelotaDisponible)
            subiendoPelota = false;

        if(!sensorArriba.get()){
            detener_dosificador();
        }

        if (subiendoPelota) {
            // mover correa
            correa_subiendo();
            detener_recogedor();



            // Revisar si llego la que estaba subiendoPelota
            if (pelotaS2) // habia una y todavia esta
            {
                pelotaS2 = !sensorMedio.get();
                acelerar_dosificador(0.20*VEL_DOSIFICADOR);

            } else {
                subiendoPelota = sensorMedio.get();
            }
        } else {
            // detener correa
            detener_correa();
            if(pelotaDisponible){
                detener_recogedor();
                ascensorActivo = false;
            } else {
                recogedor_entrando();
            }

        }


    }

    private void descargando_pelotas(){
        // SI DISPONIBLE: solo sacar
        // SI NO DISPONIBLE: bajar hasta que haya pelotaDisponible; solo si ya habia pelotas


        //TODO:  Usar un contador de pelotas (FIFO)

        boolean hayPelotas = false;
        if (!sensorArriba.get() || !sensorMedio.get()){
            hayPelotas = true;
        }

        if (pelotaDisponible){
            recogedor_saliendo();
            detener_correa();
            // bajando = false;
            contadorDescargar = 80; // 500ms @ 20ms
        } else {
            if (hayPelotas){
                correa_bajando();

            } else if (contadorDescargar > 0){
                contadorDescargar--;
            } else {

                detener_recogedor();
                descargaActivo = false;
            }
        }

    }

    /* ------- METODOS PARA BOTONES ---------- */

    private void ascensor_boton_R1() {
        if (controlDriver.getBumperPressed(Hand.kRight)){
            descargaActivo = false;
            if (ascensorActivo == false) { // ya esta apagado
                activar_ascensor();
                subiendoPelota = false;

            } else {
                detener_recogedor();
                detener_correa();
                detener_dosificador();
                ascensorActivo = false;
            }
        }
    }

    private void descarga_boton_L1() {
        if(controlDriver.getBumperPressed(Hand.kLeft)){
            if (descargaActivo == false) { // ya esta apagado
                activar_descarga();

            } else {
                detener_correa();
                detener_recogedor();
                descargaActivo = false;
            }
        }
    }

    private void detener_estados_boton_back(){
        if(controlDriver.getBackButtonPressed() || assistantDriver.getBackButtonPressed()){
            reiniciar_estados();
        }
    }

    private void bajando_boton_2A() {
        if (assistantDriver.getAButtonPressed()) {
            // if (bajandoActivo == false) { // ya esta apagado
            if (!bajandoPelotas.estaActivo()){
                // bajandoActivo = true; // corriendo
                // activar_bajando();
                bajandoPelotas.entrada();

            } else {
                // detener_correa();
                // detener_recogedor();
                // bajandoActivo = false;
                bajandoPelotas.salida();
            }
        }
    }

    private void subiendo_boton_2Y() {
        if (assistantDriver.getYButtonPressed()) {
            if (!subiendoActivo) {
                activar_subiendo();
            } else {
                detener_correa();
                detener_recogedor();
                subiendoActivo = false;
            }
        }
    }

    private void shooter_encendido_boton_X() {
        if(controlDriver.getXButtonPressed()){
            if (shooterActivo == true) {
                shooterActivo = false;
                activar_shooter_encendido();

            } else {
                shooterActivo = true;
            }
        }
    }

    private void recogedor_saliendo_boton_y(){
        if(controlDriver.getYButtonPressed()){
            if(recogedorSaliendoActivo == false){
                recogedorSaliendoActivo = true;
                recogedor_saliendo();
            } else{
                recogedorSaliendoActivo = false;
            }
        }
    }

    private void camara_alineando_boton_2X() {
        if (assistantDriver.getXButtonPressed()) {
            if (alineandoActivo == false) { // Entrar al estado
                activar_robot_alineandose();
                /* Esta Accion no puede estar dentro del metodo para Activar el estado */
                // alinear_con_camara();
            } else { // Salir del estado
                // detener_robot();
                alineandoActivo = false;
            }

        }

    }

    private void correr_dosificador_boton_2B() {
        if(assistantDriver.getBButton()){
            if (dosificadorActivo == false) {
                activar_dosificador();

            } else if (dosificadorActivo == true) {
                detener_dosificador();
                dosificadorActivo = false;

            }
        }


    }

    private void invertir_motores_boton_Y() {
        if (controlDriver.getYButtonPressed()) {
            if (motoresInvertidos == false) { // Entrar al estado
                activar_invertir();
                invertir_motores(true);
            } else { // Salir del estado
                motoresInvertidos = false;
                invertir_motores(false);
            }
        }

    }

    private void disparar_dosificador_boton_B(){
        if (controlDriver.getBButtonPressed()) {
            if (dispararDosificadorActivo == false) { // Entrar al estado
                activar_disparar_dosificador();

            }
            else{
                dispararDosificadorActivo = false;
            }
        }
    }

    private void cambiar_angulo_shooter_boton_A(){
        if(controlDriver.getAButtonPressed()){
            if(shooterInclinado == false){
                shooterInclinado = true;
                inclinarShooter(true);
            }
            else {
                inclinarShooter(false);
                shooterInclinado = false;
            }
        }
    }

    /* ------ METODOS PARA EL RECOGEDOR --- */

    private void acelerar_recogedor(double vel) {
        // recogedorMotor1.set(ControlMode.PercentOutput, vel);
        // recogedorMotor2.set(ControlMode.PercentOutput, vel);
        rearRecogedorMotor.set(ControlMode.PercentOutput, vel);
        frontRecogedorMotor.set(ControlMode.PercentOutput, vel);
    }

    private void detener_recogedor() {
        acelerar_recogedor(0.0);
    }

    /* ------ METODOS PARA LA CORREA --- */

    private void detener_correa() {
        acelerar_Correa(0.0);
    }

    private void acelerar_Correa(double vel) {
        correaFrontMotor.set(ControlMode.PercentOutput, vel *-1);
        correaBackMotor.set(ControlMode.PercentOutput, vel *-1);

    }

    /* ------ METODOS PARA EL SHOOTER --- */

    private void detener_shooter() {
        acelerar_shooter(0.0);
    }

    private void acelerar_shooter(double vel) {
        rightShooterMotor.set(ControlMode.PercentOutput, +vel);
        leftShooterMotor.set(ControlMode.PercentOutput, -vel);
    }

    private void detener_dosificador() {
        acelerar_dosificador(0.0);
    }

    private void acelerar_dosificador(double vel) {
        dosificadorMotor.set(ControlMode.PercentOutput, vel * -1);
    }

    private void dosificador_disparando(){

        // Aumenta el contador
        contador_dosificador++;
        if (contador_dosificador < 4*tiempo_dosificador) {
            acelerar_shooter(VEL_SHOOTER);
        } else if (contador_dosificador < 14*tiempo_dosificador){
            // Si hay pelota: dosificar
            // Si no hay pelota: subir una y detener el contador

            if (pelotaLista){
                acelerar_dosificador(VEL_DOSIFICADOR);

            } else {
                //Forzar un valor en el contador que nos lleve a la parte de recargar
                contador_dosificador = 14*tiempo_dosificador;
            }

        } else if (contador_dosificador < 18*tiempo_dosificador){
            correa_subiendo();
            acelerar_dosificador(0.3*VEL_DOSIFICADOR);
            detener_shooter();
            if(pelotaDisponible){
                recogedor_entrando();
            } else {
                detener_recogedor();
            }
            if(pelotaLista){
                contador_dosificador = 18*tiempo_dosificador;
            }
        } else {
            detener_correa();
            detener_dosificador();
            contador_dosificador = 0;
            // Salir del estado (deberia ser activar otro estado)
            dispararDosificadorActivo = false; //resetear solo
        }
        
    }

    /* ------- METODOS PARA MOVER EL ROBOT ----- */
    private void mover_con_joysticks(){
        if (!drivetrainBloqueado){
            acelerar_robot(controlDriver.getY(Hand.kLeft) * -1, controlDriver.getY(Hand.kRight));
        }
    }

    private void avanzar_por_velocidad(double vel) {
        acelerar_robot(vel, vel);
    }

    private void acelerar_robot(double vel_left, double vel_right) {
        leftBackDrive.set(ControlMode.PercentOutput, vel_left ) ;
        leftFrontDrive.set(ControlMode.PercentOutput, vel_left);
        rightBackDrive.set(ControlMode.PercentOutput, vel_right);
        rightFrontDrive.set(ControlMode.PercentOutput, vel_right);
    }

    private void girar_robot(double v) {
        // izquierda: motor derecho P positivo y motor izquierdo P negativo
        // derecha: motor derecho P negativo y motor izquirdo P positivo
        acelerar_robot(-1 * v, v);
    }

    private void detener_robot() {
        acelerar_robot(0, 0);
    }

    private void invertir_motores(boolean val) {
        leftFrontDrive.setInverted(val);
        leftBackDrive.setInverted(val);
        rightFrontDrive.setInverted(val);
        rightBackDrive.setInverted(val);
    }

    /* ------ METODOS NEUMATICA ------- */
    public void iniciar_neumatica() {
        mainCompressor.setClosedLoopControl(true);
        // Compressor.setClosedLoopControl(false);
        enabled = mainCompressor.enabled();
        //  Compressor.start();
        pressureSwitch = mainCompressor.getPressureSwitchValue();
        current = mainCompressor.getCompressorCurrent();
    }

    public void inclinarShooter(boolean val){
        if(val){
            shooterSolenoid.set(Value.kForward); // to-do: verificar cual de los dos inclina
        }
        else {
            shooterSolenoid.set(Value.kReverse);
        }
    }

    /* ------ METODOS CAMARA ------- */

    /*
     * Alinea el robot con un objetivo haciendo que el centro del objetivo se
     * localice en el centro (horizontal) de la imagen.
     *
     * Se toma el valor tx como "error" y se usa para calcular una Potencia de giro
     * para pasarle a los motores. El valor de esta Potencia es PROPORCIONAL al
     * valor de tx. La proporcion es dada por la constante KC.
     *
     * Esto es, mientras más separado el objetivo del centro, más fuerte se P
     * tencia cero (alineado).
     */

    private void alinear_con_camara() {
        // Este metodo se ejecuta una vez en cada periodo de TeleOp
        // if (rangeAdjustment <= 90.0) {
        // Leer el "error" (valor tx, o "delta x")
        double Dx = txLimelight.getDouble(0.0);
        Val_Dx = Dx;
        // Val_Dx++;
        // Potencia deseada segun el error
        Kp = 0.03;
        double P = Kp * Dx; // KC debe ser creada y definida con un valor adecuado
        Val_P = P;
        // P = Math.min(0.45, P);

        // if (Math.abs(P) >= 0.0) {
        // girar_robot(P);
        // } else {
        // detener_robot();
        // }
        /*
         * Hay que convertir esta potencia de giro en las potencias (velocidades) que se
         * le piden a los motores En este caso, debera ser la misma para ambos, pero con
         * signo diferente.
         */

        // en algun momento quiero anunciar que ya nos alineamos.

        // if (Math.abs(Dx) <= 1.0) { alineandoActivo = false; }
        // }

        ty_distance = ty();
        currentDistance = 57 / Math.tan(Math.toRadians(ty_distance));
        goalDistance = 130; // pulgadas
        distanceError = goalDistance - currentDistance;
        kp_dist = 0.004;
        rangeAdjustment = kp_dist * distanceError;
        rangeAdjustment = Math.min(0.5, rangeAdjustment);

        combinacionVelocidadPos = (-rangeAdjustment + P);
        combinacionVelocidadNeg = (+rangeAdjustment + P);
        combinacionVelocidadPos = Math.min(0.45, combinacionVelocidadPos);
        combinacionVelocidadNeg = Math.min(0.45, combinacionVelocidadNeg);

        if (Math.abs(combinacionVelocidadPos) >= 0.0 || Math.abs(combinacionVelocidadNeg) >= 0.0) {
            acelerar_robot(combinacionVelocidadPos, combinacionVelocidadNeg);
        } else {
            alineandoActivo = false; // Es poco probable que salga (ambos en == 0.0)
        }

    }

    private double tx() {
        // table = NetworkTableInstance.getDefault().getTable("Limelight");
        // txLimelight = table.getEntry("tx");
        // tyLimelight = table.getEntry("ty");
        // taLimelight = table.getEntry("ta");
        double alpha = 0.6;
        double val = txLimelight.getDouble(0.0);
        tx_mem = average_r(val, alpha, tx_mem);
        return tx_mem;
    }

    private double ty() {
        double alpha = 0.6;
        double val = tyLimelight.getDouble(0.0);
        if (val > 0) {
            ty_mem = average_r(val, alpha, ty_mem);
        }
        return ty_mem;
    }

    private double ta() {
        return taLimelight.getDouble(0.0);
    }

    private double average_r(double val, double alpha, double mem) {
        return alpha * val + (1 - alpha) * mem;
    }



    private class BajandoPelotas extends Estado{

        BajandoPelotas(Robot robot){
            super(robot);
        }

        public void entrada(){
            activar();
        }

        public void salida(){
            R.detener_correa();
            R.detener_recogedor();
            desactivar();
        }

        public void ejecutar(){
            R.correa_bajando();
            R.recogedor_saliendo();
        }
    }
}