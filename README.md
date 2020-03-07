### Software TF 2020
Programa modular basado principalmente en acciones y estados para definir lo que se ejectua en cada caso.

Las acciones son de muy bajo nivel, en general, y se enfocan en acciones basicas de hardware como encender o detener motores. 

Los estados son combinaciones de acciones y otros estados que se ejecutan segun las circunstancias. Algunos estados son efimeros, que hacen algunas cosas y luego salen, y otros se mantienen haciendo algo simple hasta que se detiene.

# Tirar todas las pelotas
# Acciones

## De Movimiento
- mover con joysticks 
- avanzar por velocidad
- acelerar recogedor
- girar robot
- detener robot
- invertir motores


## De las pelotas

### recogedor
- recogedor entrando
- recogedor saliendo
- acelerar recogedor
- detener recogedor


### correa
- correa subiendo
- correa bajando
- detener correa
- acelerar correa


## Del disparador
- acelerar shooter
- detener shooter 
- Separando (Dosificar)
- acelerar dosificador
- detener dosificador

### Camara
 - alinear_con_camara (eliminar accion de disparar despues de alinearse)
 - tx
 - ty
 - ta
 - average r


### Neumaticas
- initNeumatics (iniciar neumatica)
- Solenoid (cambiar angulo shooter)


## Estados 
La ejecucion de acciones o serie de acciones para poner al robot a actuar de cierta manera. El estado no tiene un objetivo en especifico.
### Activar y reiniciar estados
- reiniciar estados
- Activar robot detenido 
- activar shooter encendido
- activar recogedor saliendo
- activar subiendo (recogiendo continuo)
- activar bajando (botando continuo)
- activar robot alineandose
- activar disparar dosificador
- activar descarga
- activar ascensor
- activar separando (activando dosificar)
- activar invertir
- robot detenido (probablemente no sea necesario)
### Botar pelotas
- robot descargando (eliminar metodo)
- descargador (robot descargando)
### Tirar una pelota
-disparar dosificador
### Cargar una pelota
### Recoger y ordenar
- robot recargando (eliminar metodo)
- ascensor (robot recargando)
### Recoger sin ordenar

### Dosificador
- dosificador disparando (eliminar metodo)
- disparar dosificador (dosificador disparando)

## Tarea 
La ejecucion de una serie de estados y acciones para lograr algo en especifico con el robot, sin permitir otra cosa que no sea lo establecido en la tarea. Hay un objetivo.

### Alinearse
- alinear con camara (Sacar la accion de tirar pelotas despues de alinearse y crear una tarea a parte llamada "alinearse y tirar pelotas" para que realize esa accion y llame a alinear con camara)
### Tirar todas las pelotas
### Alinearse y tirar pelotas
- shooter encendido (alinearse y tirar pelotas)
### Colgarse