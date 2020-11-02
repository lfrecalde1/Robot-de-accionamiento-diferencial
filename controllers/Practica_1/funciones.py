import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

def Jacobiano(v,phi):
    J=np.matrix([[np.cos(phi),0],[np.sin(phi),0]])
    hp=J@v
    return hp

def euler(z,zp,t_sample):
    z=z+zp*t_sample
    return z
    
def conversion(v,r,L):
    T=np.matrix([[r/2,r/2],[r/L,-r/L]])
    tranformacion_ruedas=np.linalg.inv(T)@v
    return tranformacion_ruedas[0,0],tranformacion_ruedas[1,0]

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