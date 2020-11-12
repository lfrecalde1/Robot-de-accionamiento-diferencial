"""CONTROL_TRAYECTORIA controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import *
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
import time
import matplotlib as mpl

robot = Robot()

# get the time step of the current world.
timestep = int(95)# 100 milisegundos equivale a 0.1 segundos

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
## Parte de declaracion de los motores del robot

wheels = []
wheelsNames = ['right wheel', 'left wheel']

for i in range(2):
    wheels.append(robot.getMotor(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)
    
# DEFINCION DEL SENSOR GPS
gps = GPS("gps")
gps.enable(timestep)

# DFINCION DE LA IMU
imu = InertialUnit("inertial unit")
imu.enable(timestep)

# Definir el tiempo de sampleo del sistema
t_sample=0.1
t_final=15+t_sample
t=np.arange(0,t_final,t_sample)
t=t.reshape(1,t.shape[0])

#Parametros del robot diferencial
r=0.190/2
L=0.381
a=0.15

#velocidades generales
u=-0.2*np.ones((t.shape[0],t.shape[1]))
w=0*np.ones((t.shape[0],t.shape[1]))

#velocidades de cada rueda
w_r=0*np.ones((t.shape[0],t.shape[1]))
w_l=0*np.ones((t.shape[0],t.shape[1]))

#Posiciones del robot
x=np.zeros((t.shape[0],t.shape[1]+1))
y=np.zeros((t.shape[0],t.shape[1]+1))
phi=np.zeros((t.shape[0],t.shape[1]+1))

#DEFINICION DE LOS ERRORES DE CONTROL

herrx=np.zeros((t.shape[0],t.shape[1]))
herry=np.zeros((t.shape[0],t.shape[1]))

def grafica(sty,titulo,x,y,etiqueta,ejex,ejey,color):
    mpl.style.use(sty)
    fig, ax = plt.subplots()
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.plot(x, y, color, label=etiqueta)
    ax.grid(linestyle='--', linewidth='0.3', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show()
    
def tranformacion_cordenadas(x,y,z,phi):
    T=np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
    relativo=np.array([[x],[y],[z]])
    real=T@relativo
    return real[0,0],real[1,0],real[2,0]


if robot.step(timestep) != -1:
    #posiciones iniciales lectura del robot
    posicion = gps.getValues()
    #Tranformacion de las posiciones reales al sistema de referencia deseado
    x_real,y_real,z_real=tranformacion_cordenadas(posicion[2],posicion[0],posicion[1],np.pi+0.0001)
    phi[0,0]=imu.getRollPitchYaw()[2]
    x[0,0]=x_real+a*np.cos(phi[0,0])
    y[0,0]=y_real+a*np.sin(phi[0,0])
    


def conversion(v,r,L):
    T=np.matrix([[r/2,r/2],[r/L,-r/L]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0]
    
def controlador(h,hd,hdp,q,a,k1,k2):
    K1=k1*np.eye(2,2)
    K2=k2*np.eye(2,2)
    herr=hd-h
    J=np.matrix([[np.cos(q),-a*np.sin(q)],[np.sin(q),a*np.cos(q)]])
    control=np.linalg.inv(J)@(hdp+K2@np.tanh(np.linalg.inv(K2)@K1@herr))
    return control[0,0], control[1,0]
    
def grafica_c(sty,titulo,x,y,etiqueta,ejex,ejey,color,x_1,y_1,etiqueta_1,color_1):
    mpl.style.use(sty)
    fig, ax = plt.subplots()
    ax.set_title(titulo.format(sty), color='0')
    ax.set_xlabel(ejex)
    ax.set_ylabel(ejey)
    ax.plot(x, y, color,label=etiqueta)
    ax.plot(x_1,y_1,color_1,label=etiqueta_1)
    ax.plot()
    ax.grid(linestyle='--', linewidth='0.3', color='black')
    legend = ax.legend(loc='upper right', shadow=False, fontsize='small')
    plt.show() 
       
#Trayectoria de referencia del sistema
xd=1*np.cos(0.6*t)
yd=1*np.sin(0.6*t)

xd_p=-1*0.6*np.sin(0.6*t)
yd_p=1*0.6*np.cos(0.6*t)
# Definicion de las ganancias del controlador
k1=1
k2=0.5

for k in range(0,t.shape[1]):
    if robot.step(timestep) != -1:
        # Seccion para almacenar los valores de los errors del sistema
        herrx[0,k]=xd[0,k]-x[0,k]
        herry[0,k]=yd[0,k]-y[0,k]

        # Definicion del vector de posiciones
        h=np.array([[x[0,k]],[y[0,k]]])

        #Definicion del vector de posiciones deseadas
        hd=np.array([[xd[0,k]],[yd[0,k]]])

        # Definicion de la derivada de la posicion deseada
        hdp=np.array([[xd_p[0,k]],[yd_p[0,k]]])

        u[0,k],w[0,k]=controlador(h,hd,hdp,phi[0,k],a,k1,k2)

        ## Vector de velocidades del sistema
        v=np.array([[u[0,k]],[w[0,k]]])
        
        ## conversion de velocidades generales a velocidades angulares de cada rueda
        w_r[0,k],w_l[0,k]=conversion(v,r,L)
        
        wheels[0].setVelocity(w_r[0,k])
        wheels[1].setVelocity(w_l[0,k])
        
        posicion = gps.getValues()
        
        x_real,y_real,z_real=tranformacion_cordenadas(posicion[2],posicion[0],posicion[1],np.pi)
 
        x[0,k+1]=x_real+a*np.cos(phi[0,k])
        y[0,k+1]=y_real+a*np.sin(phi[0,k])
        phi[0,k+1]=imu.getRollPitchYaw()[2]

        print(x[0,k+1],y[0,k+1],imu.getRollPitchYaw()[2])

wheels[0].setVelocity(0)
wheels[1].setVelocity(0)
grafica_c('default','Trayectoria',x[0,:],y[0,:],'$\mathbf{\eta(t)}$','$x[m]$','$y[m]$','b',xd[0,:],yd[0,:],'$\mathbf{\eta_{d}(t)}$','g')
grafica_c('default','Trayectoria',t[0,:],herrx[0,:],'$E_x$','$t[s]$','$E$','b',t[0,:],herry[0,:],'$E_y$','g')
grafica('default','Velocidad Lineal',t[0,:],u[0,:],'$\mu(t)$','$t[s]$','$[m/s]$','g')
grafica('default','Velocidad Angular',t[0,:],w[0,:],'$\omega(t)$','$t[s]$','$[rad/s]$','r')
grafica('default','Velocidad angular rueda derecha',t[0,:],w_r[0,:],'$\omega_{r}(t)$','$t[s]$','$[rad/s]$','r')
grafica('default','Velocidad angular rueda Izquierda',t[0,:],w_l[0,:],'$\omega_{l}(t)$','$t[s]$','$[rad/s]$','b')

print("FINALIZACION DEL PROGRAMA")
# Enter here exit cleanup code.
