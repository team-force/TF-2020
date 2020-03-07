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
    final double VEL_CORREA = 0.3;
    final double VEL_SHOOTER = 0.80;
    final double VEL_RECOGEDOR = 0.25;
    final double VEL_Dosificador= 0.6;

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
    private XboxController ControlDriver = new XboxController(2);
    private XboxController AssistantDriver = new XboxController(0);

    // ---------------- FIN CONTROLES ------------ //

    // ---------------- NEUMATICA --------------- //

    private Compressor mainCompressor = new Compressor(0);
    private DoubleSolenoid shooterSolenoid = new DoubleSolenoid(4, 5);
    boolean enabled;
    boolean pressureSwitch;
    double current;
    // ------------- FIN DE NEUMATICA ----------- //

    // ---------------- SENSORES ------------ //
    private DigitalInput SensorAbajo = new  DigitalInput(7);
    private DigitalInput SensorMedio = new  DigitalInput(8);
    private DigitalInput SensorArriba = new  DigitalInput(9);
    
    // ---------------- FIN SENSORES ------------ //
    
    // ----------------- VARIABLES ---------------//

    // Banderas para cambiar entre estados
    // TODO: ponerlas en el mismo orden en que aparecen los estados en
    // TeleopPeriodic
    private boolean subiendoActivo = false;
    private boolean bajandoActivo = false;
    private boolean shooterActivo = false;
    private boolean detenidoActivo = true;
    private boolean soltandoActivo = false;
    private boolean alineandoDistanciaActivo = false;
    private boolean alineandoActivo = false;
    private boolean invertirActivo = false;
    private boolean solenoidActivo = false;
    private boolean dispararDosificadorActivo = false; //indica si activado
    private boolean separadorActivo = false; //indica si activado
    private boolean ascensorActivo = false;
    private boolean descargaActivo = false;

    private boolean drivetrainBloqueado = true;

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

    // Para el ascensor
    
    private boolean cargado = false;
    private boolean disponible = false;
    private boolean subiendo = false;
    private boolean pelotaS2 = false;
    private boolean pelotaLista = false;
    private int contador_descargar = 0;


    // ------- Variables para el Shooter ----
    private double angulo_shooter = 40; // en Grados para entender
    private double radianes_shooter = angulo_shooter / 180 * 01;
    private int tiempo_dosificador=10;
    private int contador_dosificador = 0;

    @Override
    public void robotInit() {
        
        // m_stick = new Joystick(2);
        initNeumatics();

        table = NetworkTableInstance.getDefault().getTable("limelight");
        txLimelight = table.getEntry("tx");
        tyLimelight = table.getEntry("ty");

        alineandoActivo = false;
        alineandoDistanciaActivo = false;
        invertirActivo = false;
        Kp = 0.05;
        Ki = 0.002;
        // taLimelight = table.getEntry("ta");
    }

    @Override
    public void robotPeriodic() {
        //detector();      
        SmartDashboard.putBoolean("ASCENSOR", ascensorActivo);
        SmartDashboard.putBoolean("DESCARGA", descargaActivo);
        SmartDashboard.putBoolean("DOSIFICADOR", dispararDosificadorActivo);
        SmartDashboard.putBoolean("Cargado", cargado);
        SmartDashboard.putBoolean("Disponible", disponible);
        SmartDashboard.putBoolean("Subiendo", subiendo);
        SmartDashboard.putBoolean("Pelota 1", !SensorAbajo.get());
        SmartDashboard.putBoolean("Pelota 2", !SensorMedio.get());
        SmartDashboard.putBoolean("Pelota 3", !SensorArriba.get());  
        SmartDashboard.putBoolean("Invert", invertirActivo);
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
        subiendo = false;
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
        pelotaLista = !SensorArriba.get();        
        cargado = pelotaLista && !SensorMedio.get();
        disponible = !SensorAbajo.get(); //Pelota disponible para subir

        // ------------ LEER BOTONES ------------//
        // Del Control Princpipal (driver)
        invertir_motores_boton_Y();
        shooter_encendido_boton_X();
        solenoid_boton_A();
        
        // Del Control Secundario (Asistente)
        subiendo_boton_2Y();
        bajando_boton_2A();
        // shooter_encendido_boton_X();
        dosificador_separando_boton_2B();

        // ----------- REVISAR TAREAS Y ESTADOS -----------
        
        // Las tareas generalmente serian exclusivas (solo una a la vez)

        /* Maquina para ejecutar tareas */

        // Hay estados que son exclusivos, otros que no.
        // Para los exclusivos: if .... else if 
        // Para los demas: if .... if ..

        /* Maquina para ejecutar estados exclusivos */


        /* Maquina para ejecuta estados no exclusivos */
        if (detenidoActivo) {
            robot_detenido(); // hay que declararlo
        }
        if(alineandoActivo){
            robot_alineandose();
        } else {
            robot_detenido();
            reiniciar_estados();
        }
        if(shooterActivo){
            shooter_encendido();
        } else {
            robot_detenido();
            reiniciar_estados();
        }
        if(dispararDosificadorActivo){
            dosifcador_disparando();
        }
        if(descargaActivo){
            robot_descargando();
        }
        if(ascensorActivo){
            robot_recargando();
        }
        if(subiendoActivo){
            correa_subiendo();
            recogedor_entrando();
        }
        if(bajandoActivo){
            correa_bajando();
            recogedor_saliendo();
        }
        if(separadorActivo){
            separando();
        }
        


        mover_con_joysticks();        
    }
    // ------------------------------------------

    /* ------ ESTADOS DEL ROBOT ------------- */
    // TODO: ponerlas en el mismo orden en que aparecen los estados en
    // TeleopPeriodic

    //TODO: Revisar si es continuo
    private void recogedor_entrando() {
        acelerar_recogedor(-1*VEL_RECOGEDOR);
    }
    
    //TODO: Revisar si es continuo
    private void recogedor_saliendo() {
        acelerar_recogedor(VEL_RECOGEDOR);
    }

    private void correa_subiendo() {
        acelerar_Correa(VEL_CORREA);
    }

    private void correa_bajando() {
        acelerar_Correa(-1.0 * VEL_CORREA);
    }

    private void robot_alineandose() {
        alinear_con_camara();
    }

    private void dosifcador_disparando(){
        disparar_dosificador();
    }

    private void robot_descargando(){
        descargador();
    }

    private void robot_recargando(){
        ascensor();
    }

    private void separando(){
        acelerar_dosificador(0.3*VEL_Dosificador);
    }

    private void shooter_encendido() {
        alinear_con_camara();
    }

    private void robot_detenido() {
        // acelerar_robot(ControlDriver.getY(Hand.kLeft) * -1,
        // ControlDriver.getY(Hand.kRight));
    }
    /* ----------- METODOS PARA CAMBIAR DE ESTADOS ------------- */

    private void reiniciar_estados() {
        // Estados
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        detenidoActivo = false; // Se queda????
        soltandoActivo = false;
        alineandoActivo = false;
        invertirActivo = false;
        drivetrainBloqueado = true;

        alineandoDistanciaActivo = false;
        solenoidActivo = false;
        dispararDosificadorActivo = false;
        separadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;

    }

    private void activar_robot_detenido() {
        detenidoActivo = true;

        shooterActivo = false;
        subiendoActivo = false;
        alineandoActivo = false;
        // alineandoDistanciaActivo = false;
        bajandoActivo = false;
        drivetrainBloqueado = false;
    }

    private void activar_shooter_encendido() {
        shooterActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        detenidoActivo = false; // Se queda????
        soltandoActivo = false;
        alineandoActivo = false;
        invertirActivo = false;
        drivetrainBloqueado = false;
        separadorActivo = false;
        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_recogedor_saliendo() {
        subiendoActivo = true;

        alineandoActivo = false;
        // alineandoDistanciaActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        detenidoActivo = false;
    }

    private void activar_subiendo(){
        subiendoActivo = true;

        bajandoActivo = false;
        detenidoActivo = false; // Se queda????
        soltandoActivo = false;
        dispararDosificadorActivo = false;
        separadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }
    
    private void activar_bajando(){
        bajandoActivo = true;

        subiendoActivo = false;
        detenidoActivo = false; // Se queda????
        soltandoActivo = false;
        dispararDosificadorActivo = false;
        separadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }
    
    private void activar_robot_alineandose() {
        alineandoActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        detenidoActivo = false; // Se queda????
        soltandoActivo = false;
        invertirActivo = false;
        drivetrainBloqueado = false;

        alineandoDistanciaActivo = false;
        dispararDosificadorActivo = false;
        separadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_disparar_dosificador() {
        // alineandoDistanciaActivo = false;
        dispararDosificadorActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        soltandoActivo = false;
        alineandoActivo = false;
        separadorActivo = false;
        ascensorActivo = false;
        descargaActivo = false;
}
    
    private void activar_descarga(){
        descargaActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        soltandoActivo = false;
        dispararDosificadorActivo = false;
        separadorActivo = false;
        ascensorActivo = false;
    }

    private void activar_ascensor(){
        ascensorActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        detenidoActivo = false; // Se queda????
        soltandoActivo = false;
        dispararDosificadorActivo = false;
        separadorActivo = false;
        descargaActivo = false;
    }

    private void activar_separando(){
        separadorActivo = true;
        subiendoActivo = false;
        bajandoActivo = false;
        shooterActivo = false;
        soltandoActivo = false;

        dispararDosificadorActivo = false;

        ascensorActivo = false;
        descargaActivo = false;
    }

    private void activar_invertir(){
        invertirActivo = true;
        shooterActivo = false;
        alineandoActivo = false;

        alineandoDistanciaActivo = false;
    }    


    /* -------- ESTADOS -------- */
    private void ascensor(){
        //SUBIENDO: Si llega una pelota, subirla hasta el proximo nivel (S2)
        //CARGADO: Si llega una al tercer nivel, y el segundo no esta vacio, detengo el recogedor
        
        // CARGADO: Hay una el el tercero, y una en el segundo (ya no subir mas pelotas) (puedo recoger, pero no subo)
        // SUBIENDO: Llega una a S1 y no hay en S2
        if (!cargado){
            if(disponible ){
                subiendo = true;
                if (!SensorMedio.get()){
                    pelotaS2 = true;
                } else {
                    pelotaS2 = false;
                }
            }
        }

        if (cargado && disponible)
            subiendo = false;

        if(!SensorArriba.get())
        {
            detener_dosificador();                
        }

        if (subiendo) {
            // mover correa
            correa_subiendo();
            acelerar_dosificador(0.30*VEL_Dosificador);
            
            

            // Revisar si llego la que estaba subiendo
            if (pelotaS2) // habia una y todavia esta
            {
                pelotaS2 = !SensorMedio.get();
            } else {
                subiendo = SensorMedio.get();
            }
        } else {
            // detener correa
            detener_correa();
            if(disponible){
                detener_recogedor();
                ascensorActivo = false;
            } else {
                recogedor_entrando();
            }
            
        }
        
        
    } 
    
    private void descargador(){
        // SI DISPONIBLE: solo sacar
        // SI NO DISPONIBLE: bajar hasta que haya disponible; solo si ya habia pelotas


        //TODO:  Usar un contador de pelotas (FIFO)

        boolean hayPelotas = false;
        if (!SensorArriba.get() || !SensorMedio.get()){
            hayPelotas = true;
        }

        if (disponible){
            recogedor_saliendo();
            detener_correa();
            // bajando = false;
            contador_descargar = 80; // 500ms @ 20ms
        } else {
            if (hayPelotas){
                correa_bajando();
                
            } else if (contador_descargar > 0){
                contador_descargar--;
            } else {
                
                detener_recogedor();
                descargaActivo = false;
            }
        }
        
    }

    /* ------- METODOS PARA BOTONES ---------- */

    private void ascensor_boton_START() {
        if (ControlDriver.getStartButtonPressed()){
            descargaActivo = false;
            if (ascensorActivo == false) { // ya esta apagado
                ascensorActivo = true; // corriendo
                activar_ascensor();
                robot_recargando();
                
            } else {
                detener_recogedor();
                detener_correa();
                ascensorActivo = false;
            }
        }
    }

    private void descarga_boton_BACK() {
        if(ControlDriver.getBackButtonPressed()){ 
            ascensorActivo = false;

            if (descargaActivo == false) { // ya esta apagado
                activar_descarga();
                robot_descargando();
                
            } else {
                detener_correa();
                detener_recogedor();
                descargaActivo = false;
            }
        }
    }

    private void bajando_boton_2A() {
        if (AssistantDriver.getAButtonPressed()) {
            if (bajandoActivo == false) { // ya esta apagado
                bajandoActivo = true; // corriendo
                activar_bajando();
                correa_bajando();
                recogedor_saliendo();
                
            } else {
                detener_correa();
                detener_recogedor();
                bajandoActivo = false;
            }
        }
    }

    private void subiendo_boton_2Y() {
        if (AssistantDriver.getYButtonPressed()) {
            if (!subiendoActivo) {
                activar_subiendo();
                correa_subiendo();
                recogedor_entrando();
            } else {
                detener_correa();
                detener_recogedor();
                subiendoActivo = false;
            }
        }
    }

    private void shooter_encendido_boton_X() {
        if(ControlDriver.getXButtonPressed()){
            if (shooterActivo == true) {
                shooterActivo = false;
                activar_shooter_encendido();
                shooter_encendido();
                
            } else {
                shooterActivo = true;
            }
        }
    }

    private void camara_alineando_boton_2X() {
        if (AssistantDriver.getXButtonPressed()) {
            if (alineandoActivo == false) { // Entrar al estado
                activar_robot_alineandose();
                /* Esta Accion no puede estar dentro del metodo para Activar el estado */
                // alinear_con_camara();
                robot_alineandose();
            } else { // Salir del estado
                // detener_robot();
                alineandoActivo = false;
            }

        }

    }

    private void dosificador_separando_boton_2B() {
        if(AssistantDriver.getBButton()){
            if (separadorActivo == false) {
                activar_separando();
                
            } else if (separadorActivo == true) {
                detener_dosificador();
                separadorActivo = false;
                
            }
        }
        
    
    }
    
    private void invertir_motores_boton_Y() {
        if (ControlDriver.getYButtonPressed()) {
            if (invertirActivo == false) { // Entrar al estado
                activar_invertir();
                invertir_motores(true);
            } else { // Salir del estado
                invertirActivo = false;
                invertir_motores(false);
            }
        }

    }

    private void disparar_dosificador_boton_B(){
        if (ControlDriver.getBButtonPressed()) {
            if (dispararDosificadorActivo == false) { // Entrar al estado
                activar_disparar_dosificador();
                dosifcador_disparando();
                
            }
            else{
                dispararDosificadorActivo = false;
            }
        }
    }
 
    private void solenoid_boton_A(){
        if(ControlDriver.getAButtonPressed()){
            if(solenoidActivo == false){
                solenoidActivo = true;
                Solenoid(true);
            }
            else {
                Solenoid(false);
                solenoidActivo = false;
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
        correaFrontMotor.set(ControlMode.PercentOutput, vel);
        correaBackMotor.set(ControlMode.PercentOutput, vel);

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
        dosificadorMotor.set(ControlMode.PercentOutput, vel);
    }
    
    private void disparar_dosificador(){
            if(dispararDosificadorActivo == true){
        
                // Aumenta el contador
                contador_dosificador++;
                if (contador_dosificador < 4*tiempo_dosificador) {
                    acelerar_shooter(VEL_SHOOTER);
                } else if (contador_dosificador < 14*tiempo_dosificador){
                    // Si hay pelota: dosificar
                    // Si no hay pelota: subir una y detener el contador
        
                    if (pelotaLista){
                        acelerar_dosificador(VEL_Dosificador);
                        
                    } else {
                        contador_dosificador = 14*tiempo_dosificador; //Recargar
                    }
                
                } else if (contador_dosificador < 18*tiempo_dosificador){
                    correa_subiendo();
                    acelerar_dosificador(0.3*VEL_Dosificador);
                    detener_shooter();
                    if(disponible){
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
    }

    /* ------- METODOS PARA MOVER EL ROBOT ----- */
    private void mover_con_joysticks(){
        if (drivetrainBloqueado){
            acelerar_robot(ControlDriver.getY(Hand.kLeft) * -1, ControlDriver.getY(Hand.kRight));
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
    public void initNeumatics() {
        mainCompressor.setClosedLoopControl(true);
        // Compressor.setClosedLoopControl(false);
        enabled = mainCompressor.enabled();
        //  Compressor.start();
        pressureSwitch = mainCompressor.getPressureSwitchValue();
        current = mainCompressor.getCompressorCurrent();
    }
    
    public void Solenoid(boolean val){
        if(val){
            shooterSolenoid.set(Value.kForward);
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
        } else if(Math.abs(combinacionVelocidadPos) == 0.0 && Math.abs(combinacionVelocidadNeg) == 0.0 && shooterActivo){
            dispararDosificadorActivo = true;
        } else {
            alineandoActivo = false;
            shooterActivo = false;
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


    
}