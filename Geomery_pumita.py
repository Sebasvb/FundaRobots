
import sympy as sp 
import matplotlib.pyplot as plt

# Variables que serán utilizadas
q1, q2, q3, q4, q5, q6 = sp.symbols("q1 q2 q3 q4 q5 q6")
l1, l2, l3, l4, l5,l6= sp.symbols("l1 l2 l3 l4 l5 l6")

def sTrasl(x, y, z):
    """ Transformación homogénea que representa traslación pura
    """
    T = sp.Matrix([[1,0,0,x],
                   [0,1,0,y],
                   [0,0,1,z],
                   [0,0,0,1]])
    return T

def sTrotx(ang):
    """ Transformación homogénea que representa rotación alrededor de x
    """
    T = sp.Matrix([[1, 0,0,0],
                   [0, sp.cos(ang),-sp.sin(ang),0],
                   [0, sp.sin(ang), sp.cos(ang),0],
                   [0, 0, 0, 1]])
    return T

def sTrotz(ang):
    """ Transformación homogénea que representa rotación alrededor de z
    """
    T = sp.Matrix([[sp.cos(ang),-sp.sin(ang),0,0],
                   [sp.sin(ang), sp.cos(ang),0,0],
                   [0,0,1,0],
                   [0,0,0,1]])
    return T

# Transformaciones con respecto al sistema anterior


T01 =sTrasl(0      ,       0,0.6718) * sTrotz(q1)
T12 =sTrasl(0      ,-0.13970,     0) * sTrotx(sp.pi/2) * sTrotz(q2)
T23 =sTrasl(0.4318 ,       0,     0) * sTrotz(q3)
T34 =sTrasl(0      , -0.4331,     0) * sTrotx(sp.pi/2) * sTrotz(q4)
T45 =sTrotx(sp.pi/2) * sTrotz(q5)
T56 =sTrasl(0      ,   0.0558,     0) * sTrotx(-sp.pi/2) * sTrotz(q6)



# Transformación del eslabón 6 con respecto a la base (0)
T06 = sp.simplify(T01*T12*T23*T34*T45*T56)

# Mostrar las transformaciones homogéneas (display funciona con IPython)
print("T01:"); T01
print("T12:"); T12
print("T23:"); T23
print("T34:"); T34
print("T45:"); T45
print("T56:"); T56
print("T06:"); T06

# Valor cuando todos los ángulos son cero
print("T06 cuando q=(0,0,0,0,0,0):")
# T06.subs({q1:0, q2:0, q3:0, q4:0, q5:0, q6:0}) 
# Equivalente a: T0e.subs([ (q1,0), (q2,0), (q3,0), (q4,0)])

# También se podría reemplazar las longitudes
T06.subs({q1:0, q2:0, q3:0, q4:0, l1:1, l2:1, l3:1, l4:0.5})


# ================Cálculo Numérico de la Cinemática Directa

import numpy as np

def Trotx(ang):
    """ Transformación homogénea que representa rotación en x
    """
    T = np.array([[1., 0., 0., 0.],
                  [0., np.cos(ang), -np.sin(ang), 0.],
                  [0., np.sin(ang),  np.cos(ang), 0.],
                  [0., 0., 0., 1.]])
    return T

def Troty(ang):
    """" Transformación homogénea que representa rotación en y
    """
    T = np.array([[np.cos(ang), 0., np.sin(ang), 0.],
                  [0., 1., 0., 0.],
                  [-np.sin(ang), 0., np.cos(ang), 0.],
                  [0., 0., 0., 1.]])
    return T

def Trotz(ang):
    """ Transformación homogénea que representa rotación en z
    """
    T = np.array([[np.cos(ang), -np.sin(ang), 0., 0.],
                  [np.sin(ang),  np.cos(ang), 0., 0.],
                  [0., 0., 1., 0.],
                  [0., 0., 0., 1.]])
    return T

def Trasl(x, y, z):
    """ Transformación homogénea que representa traslación pura
    """
    T = np.array([[1,0,0,x],
                  [0,1,0,y],
                  [0,0,1,z],
                  [0,0,0,1]])
    return T




# Cinemática directa del robot
def cdirecta_scara(q):
    """ Retorna los sistemas de referencia de cada eslabón con respecto a la base
    """
    # Sistemas con respecto al sistema anterior
    T01 = Trasl(0      ,       0,0.6718) @ Trotz(q[0])
    T12 = Trasl(0      ,-0.13970,     0) @ Trotx(np.pi/2) @ Trotz(q[1])
    T23 = Trasl(0.4318 ,       0,     0) @ Trotz(q[2])
    T34 = Trasl(0      , -0.4331,     0) @ Trotx(np.pi/2) @ Trotz(q[3])
    T45 = Trotx(np.pi/2) @ Trotz(q[4])
    T56 = Trasl(0      ,   0.0558,     0) @ Trotx(-np.pi/2) @ Trotz(q[5])

    # Sistemas con respecto a la base
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45
    T06 = T05 @ T56

    return T06, (T01, T02, T03, T04, T05)

# Ejemplo de cálculo de la cinemática directa

# q = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]    # Valores articulares

q=[0,0,0,0,0,0]

# q=[[],[],[],[],[],[]]
# Cinemática directa
Te, T = cdirecta_scara(q)   # Cinemática directa

# Mostrar el resultado
print("Efector final con respecto a la base cuando q1={}, q2={}, q3={}, q4={}, q5={}, q6={}".format(np.rad2deg(q[0]), np.rad2deg(q[1]),  np.rad2deg(q[2]), np.rad2deg(q[3]),  np.rad2deg(q[4]), np.rad2deg(q[5])))
print(np.round(Te,4))



# Ejemplo de cálculo de la cinemática directa q=(-pi/2,0,pi,pi/2,pi/3,pi/2)

# q = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]    # Valores articulares
q =[-np.pi/2,0,np.pi,np.pi/2,np.pi/3,np.pi/2]
# Cinemática directa
Te, T = cdirecta_scara(q)   # Cinemática directa

# Mostrar el resultado
print("Efector final con respecto a la base cuando q1={}, q2={}, q3={}, q4={}, q5={}, q6={}".format(np.rad2deg(q[0]), np.rad2deg(q[1]),  np.rad2deg(q[2]), np.rad2deg(q[3]),  np.rad2deg(q[4]), np.rad2deg(q[5])))
print(np.round(Te,4))




# Ejemplo de cálculo de la cinemática directa q=(pi/2,pi/4,pi/3,0,pi/3,-pi/4)

# q = [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0)]    # Valores articulares
q =[np.pi/2,np.pi/4,np.pi/3,0,np.pi/3,-np.pi/4]
# Cinemática directa
Te, T = cdirecta_scara(q)   # Cinemática directa

# Mostrar el resultado
print("Efector final con respecto a la base cuando q1={}, q2={}, q3={}, q4={}, q5={}, q6={}".format(np.rad2deg(q[0]), np.rad2deg(q[1]),  np.rad2deg(q[2]), np.rad2deg(q[3]),  np.rad2deg(q[4]), np.rad2deg(q[5])))
print(np.round(Te,4))



# ===============vizualizacion



def cdirecta_geom_PUMA560(q,l1,l2,l3,l4,l5):
    #Retorna los sistemas de referencia de cada eslabón con respecto a la base
    # Transformaciones homogéneas de DH
    T01 = Trasl(0,0,l1) @ Trotz(q[0])
    T12 = Trasl(0,l2,0) @ Trotx(np.pi/2) @ Trotz(q[1])
    T23 = Trasl(l3,0,0) @ Trotz(q[2])
    T34 = Trasl(0,l4,0) @ Trotx(np.pi/2) @ Trotz(q[3])
    T45 = Trotx(np.pi/2) @ Trotz(q[4])
    T56 = Trasl(0,l5,0) @ Trotx(-np.pi/2) @ Trotz(q[5])
    # Efector final con respecto a la base
    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    return T06, [T01, T01 @ T12, T01 @ T12 @ T23, T01 @ T12 @ T23 @ T34, T01 @ T12 @ T23 @ T34 @ T45, T01 @ T12 @ T23 @ T34 @ T45 @ T56]
    

def graph_PUMA560(q, l1, l2, l3, l4, l5, k=0.4):
    """ Grafica el robot según la configuración articular. Las entradas son los valores articulares, 
    las longitudes de los eslabones y un factor para el tamaño con que se muetra los sistemas de referencia
    """
    # Cálculo de la cinemática directa
    Te, T = cdirecta_geom_PUMA560(q, l1, l2, l3, l4, l5)
    # Borrar el gráfico
    # plt.clf()
    ax = plt.axes(projection='3d')
    # Nombres para los ejes
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
    # Transformaciones homogéneas con respecto a la base (ej. T2 es {2} con respecto a {0})
    T1 = T[0]; T2 = T[1]; T3 = T[2]; T4 = T[3]; T5 = T[4]
    # Cuerpo del robot
    ax.plot([0, T1[0,3]], [0, T1[1,3]], [0, T1[2,3]], linewidth=3, color='k')
    ax.plot([T1[0,3], T2[0,3]], [T1[1,3], T2[1,3]], [T1[2,3], T2[2,3]], linewidth=3, color='k')
    ax.plot([T2[0,3], T3[0,3]], [T2[1,3], T3[1,3]], [T2[2,3], T3[2,3]], linewidth=3, color='k')
    ax.plot([T3[0,3], T4[0,3]], [T3[1,3], T4[1,3]], [T3[2,3], T4[2,3]], linewidth=3, color='k')
    ax.plot([T4[0,3], T5[0,3]], [T4[1,3], T5[1,3]], [T4[2,3], T5[2,3]], linewidth=3, color='k')
    # Puntos en las articulaciones
    ax.scatter(0, 0, 0, color='g', s=50)
    # "Cilindros" para representar la dirección de las articulaciones
    ax.plot([T1[0,3]+ 0.1*T1[0,2], T1[0,3]], [T1[1,3]+0.1*T1[1,2], T1[1,3]], [T1[2,3]+0.1*T1[2,2], T1[2,3]], linewidth=5, color='g')
    ax.plot([T2[0,3]+ 0.1*T2[0,2], T2[0,3]], [T2[1,3]+0.1*T2[1,2], T2[1,3]], [T2[2,3]+0.1*T2[2,2], T2[2,3]], linewidth=5, color='g')
    ax.plot([T3[0,3]+ 0.1*T3[0,2], T3[0,3]], [T3[1,3]+0.1*T3[1,2], T3[1,3]], [T3[2,3]+0.1*T3[2,2], T3[2,3]], linewidth=5, color='g')
    ax.plot([T4[0,3]+ 0.1*T4[0,2], T4[0,3]], [T4[1,3]+0.1*T4[1,2], T4[1,3]], [T4[2,3]+0.1*T4[2,2], T4[2,3]], linewidth=5, color='g')
    ax.plot([T5[0,3]+ 0.1*T5[0,2], T5[0,3]], [T5[1,3]+0.1*T5[1,2], T5[1,3]], [T5[2,3]+0.1*T5[2,2], T5[2,3]], linewidth=5, color='g')
    # Efector final (definido por 4 puntos)
    p1 = np.array([0, 0.1, 0, 1]); p2 = np.array([0, 0.1, 0.2, 1])
    p3 = np.array([0, -0.1, 0, 1]); p4 = np.array([0, -0.1, 0.2, 1])
    p1 = Te.dot(p1); p2 = Te.dot(p2); p3 = Te.dot(p3); p4 = Te.dot(p4)
    # Sistema de referencia del efector final (con respecto al sistema 0)
    ax.plot([Te[0,3],Te[0,3]+k*Te[0,0]], [Te[1,3],Te[1,3]+k*Te[1,0]], [Te[2,3],Te[2,3]+k*Te[2,0]], color='r')
    ax.plot([Te[0,3],Te[0,3]+k*Te[0,1]], [Te[1,3],Te[1,3]+k*Te[1,1]], [Te[2,3],Te[2,3]+k*Te[2,1]], color='g')
    ax.plot([Te[0,3],Te[0,3]+k*Te[0,2]], [Te[1,3],Te[1,3]+k*Te[1,2]], [Te[2,3],Te[2,3]+k*Te[2,2]], color='b')
    # Sistema de referencia de la base (0)
    ax.plot([0,k], [0,0], [0,0], color='r')
    ax.plot([0,0], [0,k], [0,0], color='g')
    ax.plot([0,0], [0,0], [0,k], color='b')
    # Gráfico del efector final
    # ax.plot([p1[0],p2[0]], [p1[1],p2[1]], [p1[2],p2[2]], color='b', linewidth=3)
    # ax.plot([p3[0],p4[0]], [p3[1],p4[1]], [p3[2],p4[2]], color='b', linewidth=3)
    # ax.plot([p1[0],p3[0]], [p1[1],p3[1]], [p1[2],p3[2]], color='b', linewidth=3)
    # Punto de vista
    ax.view_init(elev=25, azim=45)
    # Límites para los ejes
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(0,1.2)




q = np.array([0, 0, 0, 0, 0, 0])
# q = [-np.pi/2, 0, np.pi, np.pi/2, np.pi/3, np.pi/2]
#q = [pi/2, pi/4, pi/3, 0, pi/3, -pi/4]
graph_PUMA560(q, 0.6718, -0.1397, 0.4318, -0.4331,0.0558)