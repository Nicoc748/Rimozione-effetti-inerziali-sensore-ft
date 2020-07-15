#!/usr/bin/env python
import numpy as np
import PyKDL as kdl
import rospy
from tesi.msg import Matriceacc
import serial
from geometry_msgs.msg import Twist

#Variabili globali
fx_filt = 0.0
fy_filt = 0.0 
fz_filt = 0.0
tx_filt = 0.0
ty_filt = 0.0
tz_filt = 0.0
vettorefx = []
vettorefy = []
vettorefz = []
vettoretx = []
vettorety = []
vettoretz = []
'''
alx_filt = 0.0
aly_filt = 0.0 
alz_filt = 0.0
aax_filt = 0.0
aay_filt = 0.0
aaz_filt = 0.0
vettorealx = []
vettorealy = []
vettorealz = []
vettoreaax = []
vettoreaay = []
vettoreaaz = []
vettorealxold = []
vettorealyold = []
vettorealzold = []
vettoreaaxold = []
vettoreaayold = []
vettoreaazold = []
'''
gyro_old = np.zeros(shape=(3,1))
filter_size = 17
prima = True
#first = True
acccal =  np.zeros(shape=(3,1))
acc_gyro = np.zeros(shape=(3,1))

'''
#publisheracc
arduinodata=serial.Serial('/dev/ttyUSB0',9600) 

def calibratura_acc (ax, ay, az):
    A = np.array([ax, ay, az, 1]).reshape(4,1)
    sens = np.array([0.0144, 0.0005, -0.0015, -4.8547, 0.0003, 0.0134, -0.0002, -4.9541, 0.0005, -0.0004, 0.0132, -4.7479]).reshape(3,4)
    A_calib = (np.dot(sens,A)) * 9.81
    print "Accelerazioni calibrate"
    print A_calib
    return A_calib

def calibratura_gyro (rx, ry,rz):
    V = np.array([rx,ry,rz,1]).reshape(4,1)
    mat_corr = np.array([-0.063, 0.0039, -0.001, 20.0828, 0.0003, 0.02, 0.0, -6.928, 0.0034, 0.0017, -0.0231, 10.7133]).reshape(3,4)
    G_calib = ((np.dot(mat_corr, V)) * 3.1416) / 180
    print "Gyro calibrato"
    print G_calib
    return G_calib

def publisheracc():
    global gyro_old
    global acccal
    global acc_gyro
    global first
    global alx_filt 
    global aly_filt  
    global alz_filt 
    global aax_filt 
    global aay_filt 
    global aaz_filt 
    global vettorealx 
    global vettorealy 
    global vettorealz 
    global vettoreaax 
    global vettoreaay
    global vettoreaaz
    global vettorealxold
    global vettorealyold 
    global vettorealzold 
    global vettoreaaxold 
    global vettoreaayold 
    global vettoreaazold 
    
    while (arduinodata.inWaiting()==0):
        pass
    arduinostring=arduinodata.readline()
    DataArray=" ".join(arduinostring.split())   #Eliminazione spazi vuoti
    DataArray=DataArray.split(",")
    if first:
        for j in range(filter_size):
            vettorealx.append(float(DataArray[0]))
            vettorealy.append(float(DataArray[1]))
            vettorealz.append(float(DataArray[2]))
            vettoreaax.append(float(DataArray[3]))
            vettoreaay.append(float(DataArray[4]))
            vettoreaaz.append(float(DataArray[5]))
        first = False
    if not first:
        try:
            vettorealx.pop(0)
            vettorealx.append(float(DataArray[0]))
            vettorealy.pop(0)
            vettorealy.append(float(DataArray[1]))
            vettorealz.pop(0)
            vettorealz.append(float(DataArray[2]))
            vettoreaax.pop(0)
            vettoreaax.append(float(DataArray[3]))
            vettoreaay.pop(0)
            vettoreaay.append(float(DataArray[4]))
            vettoreaaz.pop(0)
            vettoreaaz.append(float(DataArray[5]))
        except:
            vettorealx = vettorealxold
            vettorealy = vettorealyold
            vettorealz = vettorealzold
            vettoreaax = vettoreaaxold
            vettoreaay = vettoreaayold
            vettoreaaz = vettoreaazold
    vettorealxold = vettorealx
    vettorealyold = vettorealy
    vettorealzold = vettorealz
    vettoreaaxold = vettoreaax
    vettoreaayold = vettoreaay
    vettoreaazold = vettoreaaz
    alx_filt = median_filter(vettorealx)
    aly_filt = median_filter(vettorealy)
    alz_filt = median_filter(vettorealz)
    aax_filt = median_filter(vettoreaax)
    aay_filt = median_filter(vettoreaay)
    aaz_filt = median_filter(vettoreaaz)
    print "Accelerazioni filtrate"
    acccal = calibratura_acc(alx_filt, aly_filt, alz_filt)
    gyrocal = calibratura_gyro(aax_filt, aay_filt, aaz_filt)
    acc_gyro = (gyrocal - gyro_old)/0.002
    gyro_old = gyrocal
    print "Accelerazioni ottenute"
    print acccal
    print acc_gyro
    
    pub0 = rospy.Publisher('accelerazioni', Matriceacc, queue_size=10)
    msg_to_send = Matriceacc()
    msg_to_send.acc_lin_x.data = acccal[0]
    msg_to_send.acc_lin_y.data = acccal[1]
    msg_to_send.acc_lin_z.data = acccal[2]
    msg_to_send.acc_ang_x.data = acc_gyro[0]
    msg_to_send.acc_ang_y.data = acc_gyro[1]
    msg_to_send.acc_ang_z.data = acc_gyro[2]
    #rate = rospy.Rate(500)
    #rospy.loginfo(msg_to_send)
    pub0.publish(msg_to_send)
    #print "Messaggio pubblicato"
    #calcolo()
    #rate.sleep()
'''

def callbackfts(fts):
    global prima
    global fx_filt 
    global fy_filt  
    global fz_filt 
    global tx_filt 
    global ty_filt 
    global tz_filt 
    global vettorefx 
    global vettorefy 
    global vettorefz 
    global vettoretx 
    global vettorety
    global vettoretz
    if prima:
        for i in range(filter_size):   #Inizializzazione
            vettorefx.append(fts.linear.x)
            vettorefy.append(fts.linear.y)
            vettorefz.append(fts.linear.z)
            vettoretx.append(fts.angular.x)
            vettorety.append(fts.angular.y)
            vettoretz.append(fts.angular.z)
        prima = False
    if not prima:
        vettorefx.pop(0)
        vettorefx.append(fts.linear.x)
        vettorefy.pop(0)
        vettorefy.append(fts.linear.y)
        vettorefz.pop(0)
        vettorefz.append(fts.linear.z)
        vettoretx.pop(0)
        vettoretx.append(fts.angular.x)
        vettorety.pop(0)
        vettorety.append(fts.angular.y)
        vettoretz.pop(0)
        vettoretz.append(fts.angular.z)
    fx_filt = median_filter(vettorefx)
    fy_filt = median_filter(vettorefy)
    fz_filt = median_filter(vettorefz)
    tx_filt = median_filter(vettoretx)
    ty_filt = median_filter(vettorety)
    tz_filt = median_filter(vettoretz)
    print "Forze/coppie acquisite"

def callbackacc(data):
    global acccal
    global acc_gyro
    acccal[0] = data.acc_lin_x.data
    acccal[1] = data.acc_lin_y.data
    acccal[2] = data.acc_lin_z.data
    acc_gyro[0] = data.acc_ang_x.data
    acc_gyro[1] = data.acc_ang_y.data
    acc_gyro[2] = data.acc_ang_z.data
    print "Accelerazioni filtrate acquisite"

#listener
def listener():
    rospy.Subscriber('atift', Twist, callbackfts)
    rospy.Subscriber('accel_filt', Matriceacc, callbackacc)

#Filtro mediana
def median_filter(data):
    temp = []
    indexer = filter_size//2
    for b in range(len(data)):
        for z in range(filter_size):
            if b+z-indexer<0 or b+z-indexer>len(data)-1:
                temp.append(0)
            else:
                temp.append(data[b+z-indexer])
        temp.sort()       
    return temp[len(temp)//2]

def differenza_minima(a,b):
    if ((a>=0 and b>=0) or (a<0 and b<0)):
        return abs(a-b)
    else:
        return abs(a+b)

def calcolo():
    global acc_gyro
    global acccal
    global fx_filt
    global fy_filt  
    global fz_filt 
    global tx_filt 
    global ty_filt 
    global tz_filt
    rospy.init_node('calcolo', anonymous=True)
    listener()
    pub = rospy.Publisher('atift_correct', Twist, queue_size=10)
    #arduinodata.flushInput()
    while not rospy.is_shutdown():
        #publisheracc()
        hand_T_fts = kdl.Frame()
        hand_T_as = kdl.Frame()
        hand_T_fts.M = kdl.Rotation.RPY(0.0,0.0,0.0)
        hand_T_fts.p = kdl.Vector(0.0,0.0,-0.128)
        hand_T_as.M = kdl.Rotation.RPY(3.1416,0.0,0.0)
        hand_T_as.p = kdl.Vector(0.126,-0.014,-0.136)

        fts_f = kdl.Vector(fx_filt,fy_filt,fz_filt)
        print "Forze lette"
        print fts_f
        fts_tau = kdl.Vector(tx_filt,ty_filt,tz_filt)
        print "Coppie lette"
        print fts_tau

        #calcolo frame fra sensore di forza e accelerometro
        fts_T_as = kdl.Frame.Inverse(hand_T_fts) * hand_T_as
        #Estratto angoli roll-pitch-yam dalla matrice di rotazione di fts_T_as
        (r,p,y) = fts_T_as.M.GetRPY()
        #Cambio di segno a fts_T_as
        as_T_fts = kdl.Frame()
        as_T_fts.M = kdl.Rotation.RPY(-r,-p,-y)
        as_T_fts.p = -fts_T_as.p
        #Calcolo forze e coppie rispetto al frame dell'accelerometro
        as_f = kdl.Rotation.Inverse(as_T_fts.M) * fts_f
        as_p_fts = np.zeros(shape=(3,1))
        np_fts_f = np.zeros(shape=(3,1))
        for i in range(3):
            as_p_fts[i] = as_T_fts.p[i]
            np_fts_f[i] = fts_f[i]
        prodotto = as_p_fts * np_fts_f
        as_tau = kdl.Rotation.Inverse(as_T_fts.M) * (kdl.Vector(prodotto[0], prodotto[1], prodotto[2]) + fts_tau)

        as_T_cog = kdl.Frame()
        as_T_cog.M = kdl.Rotation.RPY(-3.1416, 0.0, 0.0)
        as_T_cog.p = kdl.Vector(-0.056,0.014,-0.018)
        cog_I = np.array([[0.0085,0.0,0.0],[0.0,0.002,0.0],[0.0,0.0,0.2127]])
        r = np.zeros(shape=(3,3))
        #Calcolo J
        for i in range(3):
            for j in range(3):
                r[i][j] = as_T_cog.M[i,j]
        r_x = np.array([r[0][0], r[1][0], r[2][0]]).reshape(3,1)
        r_y = np.array([r[0][1], r[1][1], r[2][1]]).reshape(3,1)
        r_z = np.array([r[0][2], r[1][2], r[2][2]]).reshape(3,1)
        J_x = np.dot((np.dot(np.transpose(r_x), cog_I)), r_x)
        J_y = np.dot((np.dot(np.transpose(r_y), cog_I)), r_y)
        J_z = np.dot((np.dot(np.transpose(r_z), cog_I)), r_z)

        m = 1.63
        as_A = np.array([acccal[0], acccal[1], acccal[2], acc_gyro[0], acc_gyro[1], acc_gyro[2]]).reshape(6,1)
        #Calcolo as_f_i
        b_x = as_T_cog.p[0]
        b_y = as_T_cog.p[1]
        b_z = as_T_cog.p[2]
        matrice_f = np.array([-m, 0, 0, 0, -m * b_z, m * b_y, 0, -m, 0, m * b_z, 0, -m * b_x, 0, 0, -m, -m * b_y, m * b_x, 0]).reshape(3,6)
        print "Matrice forze"
        print matrice_f
        np_as_f_i = np.dot(matrice_f, as_A)
        print "Forze d'inerzia ottenute"
        print np_as_f_i
        #Calcolo as_tau_i
        matrice_tau = np.array([0,m*b_z,-m*b_y,-J_x-m*(b_y**2+b_z**2),0,0,-m*b_z,0,m*b_x,0,-J_y-m*(b_x**2+b_z**2),0,m*b_y,-m*b_z,0,0,0,-J_z-m*(b_x**2+b_y**2)]).reshape(3,6)
        print "Matrice coppie"
        print matrice_tau
        np_as_tau_i = np.dot(matrice_tau, as_A)
        print "Coppie d'inerzia ottenute"
        print np_as_tau_i

        #Calcolo fts_F_i utilizzando la transformazione inversa quindi rispetto al frame del sensore di forza
        as_f_i = kdl.Vector(np_as_f_i[0],np_as_f_i[1],np_as_f_i[2])
        as_tau_i = kdl.Vector(np_as_tau_i[0],np_as_tau_i[1],np_as_tau_i[2])
        fts_f_i = as_T_fts.M * as_f_i
        np_fts_f_i = np.zeros(shape=(3,1))
        as_p_cog = np.zeros(shape=(3,1))
        for k in range(3):
            np_fts_f_i[k] = fts_f_i[k]
            as_p_cog[k] =  as_T_cog.p[k]
        prodotto2 = np_fts_f_i * as_p_cog
        fts_tau_i = kdl.Vector(prodotto2[0], prodotto2[1], prodotto2[2]) + as_T_fts.M * as_tau_i
        print "Calcolo eseguito"
        fts_F_i = Twist()
        fts_F_i.linear.x = differenza_minima(fts_f[0], fts_f_i[0])
        fts_F_i.linear.y = differenza_minima(fts_f[1], fts_f_i[1])
        fts_F_i.linear.z = differenza_minima(fts_f[2], fts_f_i[2])
        fts_F_i.angular.x = differenza_minima(fts_tau[0], fts_tau_i[0])
        fts_F_i.angular.y = differenza_minima(fts_tau[1], fts_tau_i[1])
        fts_F_i.angular.z = differenza_minima(fts_tau[2], fts_tau_i[2])
        
        rospy.loginfo(fts_F_i)
        pub.publish(fts_F_i)
        
        rate = rospy.Rate(30)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        calcolo()
    except rospy.ROSInterruptException:
        pass