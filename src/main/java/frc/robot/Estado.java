/** Clase para implementar un estado, basandose en un Robot.
 *  Esto requiere que el Robot tenga la mayoria de sus acciones
 *  marcadas como Publicas.
 */

package frc.robot;

abstract class Estado{

    protected Robot R;
    protected boolean activo;


    // Constructor
    // Estado(){
    //     activo = false;
    // }
    Estado(Robot robot){
        R = robot;
        activo = false;
    }

    // Activador
    public void activar(){
        activo = true;
    }

    // Desactivador
    public  void desactivar(){
        activo = false;
    }

    // Indicador
    public boolean estaActivo(){
        return activo;
    }

    public abstract void ejecutar();
    public abstract void entrada();
    public abstract void salida();

    
}