


import sympy as sp                # Biblioteca para el cálculo simbólico

def sTdh(d, th, a, alpha):
    cth = sp.cos(th); sth = sp.sin(th)
    ca = sp.cos(alpha); sa = sp.sin(alpha)
    Tdh = sp.Matrix([[cth, -ca*sth,  sa*sth, a*cth],
                     [sth,  ca*cth, -sa*cth, a*sth],
                     [0,        sa,     ca,      d],
                     [0,         0,      0,      1]])
    return Tdh




# Variables simbólicas
q1, q2, q3, q4, q5, q6 = sp.symbols("q1 q2 q3 q4 q5 q6")
l1, l2, l3, l4 = sp.symbols("l1 l2 l3 l4")

# Transformaciones homogéneas
T01 = sTdh( 0.6718,   q1, 0, sp.pi/2)
T12 = sTdh(   0.1397, q2+sp.pi,  -0.4318,       0)
T23 = sTdh(   0,   q3+sp.pi, 0, sp.pi/2)
T34 = sTdh( 0.4331,   q4,    0, sp.pi/2)
T45 = sTdh(   0,   q5+sp.pi,    0, sp.pi/2)
T56 = sTdh(   0.0558,         sp.pi+q6,    0,       0)

# Transformación homogénea final
Tf = sp.simplify(T01*T12*T23*T34*T45*T56)

# Mostrar las transformaciones homogéneas (display funciona con IPython)
print("T01:"); T01
print("T12:"); T12
print("T23:"); T23
print("T34:"); T34
print("T45:"); T45
print("T56:"); T56
print("T06:"); Tf



# Valor cuando todos los ángulos son cero
print("T06 cuando q=(0,0,0,0,0,0):")
Tf.subs({q1:0., q2:0., q3:0., q4:0., q5:0., q6:0.})



# Valor cuando todos los ángulos son q=(-pi/2,0,pi,pi/2,pi/3,pi/2)
print("T06 cuando q=(-pi/2,0,pi,pi/2,pi/3,pi/2):")
Tf.subs({q1:-sp.pi/2, q2:0., q3:sp.pi, q4:sp.pi/2, q5:sp.pi/3, q6:sp.pi/2})



# Valor cuando todos los ángulos son q=(pi/2,pi/4,pi/3,0,pi/3,-pi/4)
print("T06 cuando q=(pi/2,pi/4,pi/3,0,pi/3,-pi/4):")
Tf.subs({q1:sp.pi/2, q2:sp.pi/4, q3:sp.pi/3, q4:0, q5:sp.pi/3, q6:-sp.pi/4})





# ===================calculo numerico


import numpy as np

# Evitar usar notación científica al mostrar los resultados
np.set_printoptions(suppress=True)


def Tdh(d, th, a, alpha):
    """Retorna la trasformación homogénea asociada con los parámetros DH
    """
    cth = np.cos(th);    sth = np.sin(th)
    ca = np.cos(alpha);  sa = np.sin(alpha)
    Tdh = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                    [sth,  ca*cth, -sa*cth, a*sth],
                    [0,        sa,     ca,      d],
                    [0,         0,      0,      1]])
    return Tdh



def cdirecta_pUMITA(q):
    """ Retorna los sistemas de referencia de cada eslabón con respecto a la base
    """
    # Transformaciones homogéneas de DH
    T01 = Tdh(  0.6718,          q[0],        0, np.pi/2);
    T12 = Tdh(   0.1397,   q[1]+np.pi,  -0.4318,       0);
    T23 = Tdh(   0,   q[2]+np.pi,        0, np.pi/2);
    T34 = Tdh(   0.4331,         q[3],        0, np.pi/2);
    T45 = Tdh(        0,   q[4]+np.pi,        0, np.pi/2);
    T56 = Tdh(   0.0558,   q[5]+np.pi,        0,       0);
    # Efector final con respecto a la base
    Tf = T01.dot(T12).dot(T23).dot(T34).dot(T45).dot(T56)
    return Tf

# Vector articular
q = np.array([0., 0., 0., 0., 0., 0.])

# Cinemática directa
Te = cdirecta_pUMITA(np.deg2rad(q))

# Mostrar el resultado
print("Efector final con respecto a la base cuando q1={}, q2={}, q3={}, q4={}, q5={}, q6={}".format(q[0], q[1], q[2],
                                                                                                    q[3], q[4], q[5]))
print(Te)


# Valor cuando todos los ángulos son q=(-pi/2,0,pi,pi/2,pi/3,pi/2)

q = np.array([-np.pi/2,0,np.pi,np.pi/2,np.pi/3,np.pi/2])

# Cinemática directa
Te = cdirecta_pUMITA(np.deg2rad(q))

# Mostrar el resultado
print("Efector final con respecto a la base cuando q1={}, q2={}, q3={}, q4={}, q5={}, q6={}".format(q[0], q[1], q[2],
                                                                                                    q[3], q[4], q[5]))
print(Te)


# Valor cuando todos los ángulos son q=(pi/2,pi/4,pi/3,0,pi/3,-pi/4)



q = np.array([np.pi/2,np.pi/4,np.pi/3,0,np.pi/3,-np.pi/4])

# Cinemática directa
Te = cdirecta_pUMITA(np.deg2rad(q))

# Mostrar el resultado
print("Efector final con respecto a la base cuando q1={}, q2={}, q3={}, q4={}, q5={}, q6={}".format(q[0], q[1], q[2],
                                                                                                    q[3], q[4], q[5]))
print(Te)



#======================Visualización de la cinemática del Robot

from serialrobot import *

# Parámetros DH del robot pumita: d, th, a, alpha

# Transformaciones homogéneas de DH
L =[[  0.6718,    0,        0, np.pi/2,'r'],
[   0.1397,   np.pi,  -0.4318,       0,'r'],
[   0,   np.pi,        0, np.pi/2,'r'],
[   0.4331,       0,        0, np.pi/2,'r'],
[        0,   np.pi,        0, np.pi/2,'r'],
[   0.0558,   np.pi,        0,       0,'r']]

# Creación del robot
pUMITA = SerialRobot(L, name='pUMI560')

T = pUMITA.fkine([0,0,0,0,0,0], verbose=False)

np.set_printoptions(suppress=True)
print(T)


# ==============Visualización:


fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection="3d")

# Límites fijos del gráfico
alims = [[-0.1,1.3],[-0.7,0.7],[-0.1, 1.4]]

# Gráfico del robot

# pUMITA.plot(ax, [0, 0, 0, 0, 0, 0], axlimits=alims, ascale=0.3, ee=False)

# Valor cuando todos los ángulos son q=(-pi/2,0,pi,pi/2,pi/3,pi/2)
# pUMITA.plot(ax, [-np.pi/2, 0, np.pi, np.pi/2, np.pi/3, np.pi/2], axlimits=alims, ascale=0.3, ee=False)

# Valor cuando todos los ángulos son q=(pi/2,pi/4,pi/3,0,pi/3,-pi/4)
pUMITA.plot(ax, [np.pi/2, np.pi/4,np.pi/3,0,np.pi/3,-np.pi/4], axlimits=alims, ascale=0.3, ee=False)