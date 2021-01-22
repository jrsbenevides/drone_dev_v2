#!/usr/bin/env python

import math
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty, UInt8
import rospy
import numpy as np
from tf.transformations import euler_from_quaternion

class globalPlanner:
    def __init__(self):
        self.curr_pos = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        self.ROTACAO_ATUAL = 0
        self.pontoAtual = {"x": 0, "y": 0, "z": 0}
        self.alvosBloqueados = []
        self.rotas = {"x": [0, 2.75, 1, -3.75, -90], "y": [0, 0, -3.5, 3.5, -90], "z": [2, 2, 2, 2, 2, -90], "yaw": [0, 0, 0, 0, 0, 0]}
        self.pos = 1
        self.ROTINA_DIEGO = 0
        self.LIMIAR_PRECISAO_ALVO = 0.8
        self.focoCamera = {"x": 595.035494, "y": 594.674498} 
        self.ROTACIONAR = 1
        self.ajusteTempo = 5
        self.viconOne = 0
        self.pose_home = {"x": 1, "y": 0.75, "z": 0.5}
        self.psi, self.po, self.r = 0, 0, 0
        self.direcaoOrbs = 1 # 1 = esquerda e -1 = direita
        self.anguloCameraGrau = 40
        self.statusNovo = -1

        self.startSM = 0
        self.takeOffBusy = 0
        self.estado = 0
        self.stopBusy = 0
        self.timeStop = 0
        self.timeTakeOff = 0
        self.timeRecOrb = 0
        self.timePorbs = 0
        self.timeVarredura = 0
        self.timeBaseBaixo = 0
        self.ROTINA_ORBS = 0
        self.PERDEU_ORBS = 0
        self.VARREDURA = 0
        self.usingOrbs = 0
        self.uniqueBaseBaixo = 0

        self.tempoParar = 1
        self.tempoTakeOff = 6
        self.alturaDrone = 2.7
        self.tempoRecOrb = 5
        self.tempoVarredura = 5
        self.tempoBaseBaixo = 5
        self.tempoPOrbs = 6

        self.idle = 0
        self.vai = 10
        self.land = 30
        self.desFinal = 77
        self.calibraAlvo = 88
        self.abort = 100

        self.idle = 0
        self.busy = 15
        self.cheguei = 10
        self.abortei = 5
        self.ackParam = 3
        self.iniciouOrb = 0
        self.perdeu = 1
        self.semEscala = 2
        self.comEscala = 3

        self.parteFrente = 110
        self.armazenar = 120
        self.baseCosteira = 130
        self.parteBaixo = 140
        self.baseToda = 150

        self.u = []
        self.v = []
        self.area = []
        self.yaw = []
        self.m = []
        self.flag = []

        self.b1 = [{"u": [], "v": [], "area": [], "yaw": [], "m": []},{"u": [], "v": [], "area": [], "yaw": [], "m": []}]
        self.b2 = [{"u": [], "v": [], "area": [], "yaw": [], "m": []},{"u": [], "v": [], "area": [], "yaw": [], "m": []}]
        self.b3 = [{"u": [], "v": [], "area": [], "yaw": [], "m": []},{"u": [], "v": [], "area": [], "yaw": [], "m": []}]
        self.b4 = [{"u": [], "v": [], "area": [], "yaw": [], "m": []},{"u": [], "v": [], "area": [], "yaw": [], "m": []}]
        self.b5 = [{"u": [], "v": [], "area": [], "yaw": [], "m": []},{"u": [], "v": [], "area": [], "yaw": [], "m": []}]

        self.target = [{"b1": {"x": 0, "y": 0, "z": 0}, "b2": {"x": 0, "y": 0, "z": 0}, "b3": {"x": 0, "y": 0, "z": 0}, "b4": {"x": 0, "y": 0, "z": 0}, "b5": {"x": 0, "y": 0, "z": 0}}, {"b1": {"x": 0, "y": 0, "z": 0}, "b2": {"x": 0, "y": 0, "z": 0}, "b3": {"x": 0, "y": 0, "z": 0}, "b4": {"x": 0, "y": 0, "z": 0}, "b5": {"x": 0, "y": 0, "z": 0}}]
        self.targetFinal = {"b1": {"x": 0, "y": 0, "z": 0, "euc": 0}, "b2": {"x": 0, "y": 0, "z": 0, "euc": 0}, "b3": {"x": 0, "y": 0, "z": 0, "euc": 0}, "b4": {"x": 0, "y": 0, "z": 0, "euc": 0}, "b5": {"x": 0, "y": 0, "z": 0, "euc": 0}}
        self.targetPixel = {"x": 0, "y": 0}

        self.teste = 0
        self.tt = 0
        self.tempolimite = 5

        self.cmd = rospy.Publisher("planejamento", PoseStamped, queue_size=10)
        self.iniciaOrb = rospy.Publisher("iniciaOrbSlam", UInt8, queue_size=10)
        self.iniciaLog = rospy.Publisher("iniciaLog", UInt8, queue_size=10)
        self.myCamera   = rospy.Publisher("/bebop/camera_control", Twist, queue_size = 1)
        self.sobe = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.desce = rospy.Publisher("/bebop/land", Empty, queue_size=10)

        self.statusOdometry_sub = rospy.Subscriber("/bebop/odom", Odometry, self.callbackOdom)
        self.statusLog_sub = rospy.Subscriber("/scale/log", Odometry, self.callbackLog)
        self.statusPlanning_sub = rospy.Subscriber("statusPlanning", UInt8, self.callbackPosicao)
        self.statusVisao_sub = rospy.Subscriber("/perception/target", PoseArray, self.callbackVisao)
        self.statusOdom_sub = rospy.Subscriber("/bebop/odom", Odometry, self.callbackOdom)
        self.statusLog_sub = rospy.Subscriber("/scale/log", Odometry, self.callbackLog)
        
        #self.vicon_sub = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, self.callbackVicon)
    
        rospy.init_node('Global_Planner', anonymous=True)

        
        self.dormir(t=2)
        self.tt = self.time()
        self.startSM = 1
        print("comecou")

    def dormir(self, t=5):
        rospy.sleep(t)

    def time(self):
        return rospy.get_rostime().secs + (rospy.get_rostime().nsecs*1e-9)

    def toRadian(self, grau):
        return grau * math.pi / 180

    def toGrau(self, radiano):
        return radiano * 180 / math.pi

    def deftempo(self, objx, objy, objz):
        euc = math.sqrt(pow(objx, 2) + pow(objy, 2) + pow(objz, 2))

        return euc * self.ajusteTempo
    
    def euclidian(self, x, y, z, x1, y1, z1):
        return math.sqrt(math.pow(x - x1, 2) + math.pow(y - y1, 2) + math.pow(z - z1, 2))

    def createMapa(self, x1=0, y1=0, z1=0, x2=0, y2=0, z2=0, x3=0, y3=0, z3=0, x4=0, y4=0, z4=0, x5=0, y5=0, z5=0):
        desx = [x1, x2, x3, x4, x5]
        desy = [y1, y2, y3, y4, y5]
        desz = [z1, z2, z3, z4, z5]
     
        return desx, desy, desz

    def bestWay(self, desx, desy, desz):
        aux = []
        p = [self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]]
        lpx = [p[0]]
        lpy = [p[1]]
        lpz = [p[2]]
        f1 = []
        f2 = []
        f3 = []

        for j in range(5):
            for i in range(len(desx)):
                aux.append(self.euclidian(desx[i], desy[i], desz[i], lpx[j], lpy[j], lpz[j])) 
        
            b = aux.index(min(aux))
            lpx.append(desx[b])
            lpy.append(desy[b])
            lpz.append(desz[b])
            desx.remove(desx[b])
            desy.remove(desy[b])
            desz.remove(desz[b])
            aux = []

        lpx.remove(p[0])
        lpy.remove(p[1])
        lpz.remove(p[2])

        try:
            a = lpx.index(0)
            f1 = lpx[:a]
            f2 = lpy[:a]
            f3 = [self.alturaDrone, self.alturaDrone, self.alturaDrone, self.alturaDrone, self.alturaDrone][:a]
        except:
            pass
            
        if len(f1) > 0:
            return f1, f2, f3
        else:
            return lpx, lpy, [self.alturaDrone, self.alturaDrone, self.alturaDrone, self.alturaDrone, self.alturaDrone]

    def rotacionar(self, angle, t=7):
        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0
        pose.pose.orientation.x = angle
        if abs(angle) - (math.pi/2) < 0.1:
            pose.pose.orientation.y = t
        else:
             pose.pose.orientation.y = t * self.toGrau(angle) / 90
        pose.pose.orientation.z = self.vai
        print("Girando por " + str(pose.pose.orientation.y) + "segundos")
        self.estado == 15
        self.cmd.publish(pose)

    def goToLocal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
        flag = self.vai if flag == 10 else flag
        angle = self.curr_pos["yaw"]#self.toRadian(self.ROTACAO_ATUAL) 

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        if yaw == 0:
            pose.pose.orientation.x = angle - self.curr_pos["yaw"]
        else:
            pose.pose.orientation.x = yaw
        pose.pose.orientation.y = self.deftempo(x, y, z) if t == 0 else t
        pose.pose.orientation.z = flag

        if self.ROTINA_DIEGO != 5 and flag == self.vai:
            #rint("Indo para o ponto: " + str(pos_local))
            print("tempo: " + str(pose.pose.orientation.y))

        self.cmd.publish(pose)

    def goToGlobal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
        x1 = x - self.curr_pos["x"]
        y1 = y - self.curr_pos["y"]
        z1 = z - self.curr_pos["z"] 
        psi0 = self.curr_pos["yaw"]
        yawLocal = yaw - psi0
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
        

        self.goToLocal(pos_local[0], pos_local[1], pos_local[2], 0, t, flag)

    def checarAlvo(self, destino):
        bloqueado = 0
        a2 = self.deftempo(destino[0], destino[1], 0) / self.ajusteTempo
        
        for i in range(len(self.alvosBloqueados)):
            a1 = self.deftempo(self.alvosBloqueados[i][0], self.alvosBloqueados[i][1], 0) / self.ajusteTempo
            
            if abs(a1 - a2) < self.LIMIAR_PRECISAO_ALVO:
                bloqueado += 1

        if bloqueado == 0:
            return True
        else:
            return False 

    # def callbackVicon(self, vicon):
    #     pv = vicon.transform.translation
    #     ov = vicon.transform.rotation
    #     roll, pitch, yaw = euler_from_quaternion([ov.x, ov.y, ov.z, ov.w])        
    #     if self.viconOne == 0:
    #         self.psi0 = yaw
    #         self.po = pv
    #         self.viconOne = 1
    #     else:
    #         self.r = [[np.cos(self.psi0), np.sin(self.psi0) * -1, 0], [np.sin(self.psi0), np.cos(self.psi0), 0], [0, 0, 1]]
    #         posicao = np.dot(np.transpose(np.asarray(self.r)), np.asarray([[pv.x - self.po.x], [pv.y - self.po.y], [pv.z - self.po.z]]))
    #         self.curr_pos["x"] = posicao[0]
    #         self.curr_pos["y"] = posicao[1]
    #         self.curr_pos["z"] = posicao[2]
    #         self.curr_pos["yaw"] = yaw - self.psi0

    def callbackLog(self, log):
        if self.usingOrbs == 1:
            self.curr_pos["x"] = log.pose.pose.position.x
            self.curr_pos["y"] = log.pose.pose.position.y
            self.curr_pos["z"] = log.pose.pose.position.z
            roll, pitch, yaw = euler_from_quaternion([log.pose.pose.orientation.x, log.pose.pose.orientation.y, log.pose.pose.orientation.z, log.pose.pose.orientation.w])
            self.curr_pos["yaw"] = yaw

    def callbackOdom(self, odom):
        if self.usingOrbs == 0:
            self.curr_pos["x"] = odom.pose.pose.position.x
            self.curr_pos["y"] = odom.pose.pose.position.y
            self.curr_pos["z"] = odom.pose.pose.position.z
            self.curr_pos["yaw"] = odom.pose.pose.orientation.z

    def goToHome(self):
        # vai em que angulo para a home?
        self.goToGlobal(self.pose_home["x"], self.pose_home["y"], self.pose_home["z"])
        self.estado = 5

    def checkRange(self, media, valor, offset=1): # Falta calibrar offset
        if media < 12 or np.isnan(media):
            if np.isnan(media):
                return True
            else:
                if valor >= media - offset and valor <= media + offset:
                    return True
                else:
                    return False

    def ajeitaValores(self, var):
        for i in range(len(self.u)):
            if self.checkRange(np.mean(self.b1[var]["m"]), self.m[i]):
                self.b1[var]["u"].append(self.u[i])
                self.b1[var]["v"].append(self.v[i])
                self.b1[var]["area"].append(self.area[i])
                self.b1[var]["yaw"].append(self.yaw[i])
                self.b1[var]["m"].append(self.m[i])
            elif self.checkRange(np.mean(self.b2[var]["m"]), self.m[i]):
                self.b2[var]["u"].append(self.u[i])
                self.b2[var]["v"].append(self.v[i])
                self.b2[var]["area"].append(self.area[i])
                self.b2[var]["yaw"].append(self.yaw[i])
                self.b2[var]["m"].append(self.m[i])
            elif self.checkRange(np.mean(self.b3[var]["m"]), self.m[i]):
                self.b3[var]["u"].append(self.u[i])
                self.b3[var]["v"].append(self.v[i])
                self.b3[var]["area"].append(self.area[i])
                self.b3[var]["yaw"].append(self.yaw[i])
                self.b3[var]["m"].append(self.m[i])
            elif self.checkRange(np.mean(self.b4[var]["m"]), self.m[i]):
                self.b4[var]["u"].append(self.u[i])
                self.b4[var]["v"].append(self.v[i])
                self.b4[var]["area"].append(self.area[i])
                self.b4[var]["yaw"].append(self.yaw[i])
                self.b4[var]["m"].append(self.m[i])
            elif self.checkRange(np.mean(self.b5[var]["m"]), self.m[i]):
                self.b5[var]["u"].append(self.u[i])
                self.b5[var]["v"].append(self.v[i])
                self.b5[var]["area"].append(self.area[i])
                self.b5[var]["yaw"].append(self.yaw[i])
                self.b5[var]["m"].append(self.m[i])

        try:
            if len(self.b1[var]["u"]) > 0:
                cateto = np.mean(self.b1[var]["m"]) * np.cos(self.toRadian(self.anguloCameraGrau))
                self.target[var]["b1"]["x"] = cateto * np.cos(np.mean(self.b1[var]["yaw"])) 
                self.target[var]["b1"]["y"] = cateto * np.sin(np.mean(self.b1[var]["yaw"])) 
                self.target[var]["b1"]["z"] = math.sqrt(math.pow(np.mean(self.b2[var]["m"]), 2) - math.pow(cateto, 2))
        except:
            pass

        try:
            if len(self.b2[var]["u"]) > 0:
                cateto = np.mean(self.b2[var]["m"]) * np.cos(self.toRadian(self.anguloCameraGrau))
                self.target[var]["b2"]["x"] = cateto * np.cos(np.mean(self.b2[var]["yaw"])) 
                self.target[var]["b2"]["y"] = cateto * np.sin(np.mean(self.b2[var]["yaw"])) 
                self.target[var]["b2"]["z"] = math.sqrt(math.pow(np.mean(self.b2[var]["m"]), 2) - math.pow(cateto, 2))
        except:
            pass

        try:
            if len(self.b3[var]["u"]) > 0:
                cateto = np.mean(self.b3[var]["m"]) * np.cos(self.toRadian(self.anguloCameraGrau))
                self.target[var]["b3"]["x"] = cateto * np.cos(np.mean(self.b3[var]["yaw"])) 
                self.target[var]["b3"]["y"] = cateto * np.sin(np.mean(self.b3[var]["yaw"])) 
                self.target[var]["b3"]["z"] = math.sqrt(math.pow(np.mean(self.b2[var]["m"]), 2) - math.pow(cateto, 2))
        except:
            pass

        try:
            if len(self.b4[var]["u"]) > 0:
                cateto = np.mean(self.b4[var]["m"]) * np.cos(self.toRadian(self.anguloCameraGrau))
                self.target[var]["b4"]["x"] = cateto * np.cos(np.mean(self.b4[var]["yaw"])) 
                self.target[var]["b4"]["y"] = cateto * np.sin(np.mean(self.b4[var]["yaw"])) 
                self.target[var]["b4"]["z"] = math.sqrt(math.pow(np.mean(self.b2[var]["m"]), 2) - math.pow(cateto, 2))
        except:
            pass

        try:
            if len(self.b5[var]["u"]) > 0:
                cateto = np.mean(self.b5[var]["m"]) * np.cos(self.toRadian(self.anguloCameraGrau))
                self.target[var]["b5"]["x"] = cateto * np.cos(np.mean(self.b5[var]["yaw"])) 
                self.target[var]["b5"]["y"] = cateto * np.sin(np.mean(self.b5[var]["yaw"])) 
                self.target[var]["b5"]["z"] = math.sqrt(math.pow(np.mean(self.b2[var]["m"]), 2) - math.pow(cateto, 2))
        except:
            pass

        self.u = []
        self.v = []
        self.area = []
        self.yaw = []
        self.m = []

    def mesclarBases(self):
        a = []
        for i in range(2):
            for j in range(5):
                if i == 0:
                    a.append({"x": self.target[i]["b"+str(j+1)]["x"], "y": self.target[i]["b"+str(j+1)]["y"], "z": self.target[i]["b"+str(j+1)]["z"]})
                elif i == 1:
                    a.append({"x": self.target[i]["b"+str(j+1)]["y"], "y": self.target[i]["b"+str(j+1)]["x"] * -1, "z": self.target[i]["b"+str(j+1)]["z"]})
                
            
        for i in range(10):
            a[i]["euc"] = math.sqrt(math.pow(a[i]["x"], 2) + math.pow(a[i]["y"], 2))

        aux = []
        b = []

        for i in range(len(a)):
            aux.append(a[i]["euc"])

        sort = sorted(aux)

        for i in range(len(a)):
            for j in range(len(a)):
                if a[j]["euc"] == sort[i]:
                    b.append(a[j])

        # Mesclar vetores
        for j in range(5):
            for i in range(len(b)):
                if abs(self.targetFinal["b"+str(j+1)]["euc"] - b[i]["euc"]) < 1:
                    self.targetFinal["b"+str(j+1)]["x"] = (self.targetFinal["b"+str(j+1)]["x"] + b[i]["x"])/2
                    self.targetFinal["b"+str(j+1)]["y"] = (self.targetFinal["b"+str(j+1)]["y"] + b[i]["y"])/2
                    self.targetFinal["b"+str(j+1)]["z"] = (self.targetFinal["b"+str(j+1)]["z"] + b[i]["z"])/2
                else:
                    self.targetFinal["b"+str(j+1)]["x"] = b[i]["x"]
                    self.targetFinal["b"+str(j+1)]["y"] = b[i]["y"]
                    self.targetFinal["b"+str(j+1)]["z"] = b[i]["z"]
                 
        # passar para global
        for i in range(5):
            self.targetFinal["b"+str(i+1)]["x"] += self.curr_pos["x"] + 1
            self.targetFinal["b"+str(i+1)]["y"] += self.curr_pos["y"] + 0.75
            self.targetFinal["b"+str(i+1)]["z"] += self.curr_pos["z"] + 0.5


        # Definir menor array
        dx, dy, dz = self.createMapa(self.targetFinal["b1"]["x"], self.targetFinal["b1"]["y"], self.targetFinal["b1"]["z"], self.targetFinal["b2"]["x"], self.targetFinal["b2"]["y"], self.targetFinal["b2"]["z"], self.targetFinal["b3"]["x"], self.targetFinal["b3"]["y"], self.targetFinal["b3"]["z"], self.targetFinal["b4"]["x"], self.targetFinal["b4"]["y"], self.targetFinal["b4"]["z"], self.targetFinal["b5"]["x"], self.targetFinal["b5"]["y"], self.targetFinal["b5"]["z"])
        lx, ly, lz = self.bestWay(dx, dy, dz)

        for i in range(5):
            if lx[i] != 0:
                np.insert(np.asarray(self.rotas["x"]), i+1, lx[i])
                np.insert(np.asarray(self.rotas["y"]), i+1, ly[i])
                np.insert(np.asarray(self.rotas["z"]), i+1, self.alturaDrone)
                np.insert(np.asarray(self.rotas["yaw"]), i+1, 0)
        
        if self.targetFinal["b5"]["x"] == 0:
            # Altura precisa estar a 3.2, senao tiver a pelo menos 3.2 e tiver uma base antes da suspensa comenta esse loop
            np.insert(np.asarray(self.rotas["x"]), 2, 5)
            np.insert(np.asarray(self.rotas["y"]), 2, 0.5)
            np.insert(np.asarray(self.rotas["z"]), 2, self.alturaDrone)
            np.insert(np.asarray(self.rotas["yaw"]), 2, 0)


    def callbackVisao(self, target):
        # u - v - area - pitch - yaw - m - flag
        # quando for mandar em pixel checar se o y ta confiavel pelo orbs

        # if self.teste == 0:
        #     self.tt = self.time()

        #     self.teste = 1

        # if self.time() - self.tt < 5:
        #     print(self.time() - self.tt)
        #     for pose in target.poses:
        #         if pose.orientation.w == 0:
        #             if pose.orientation.z < 7 or (pose.orientation.z * -1) > 0: # Tirar sinal de negativo
        #                 self.u.append(pose.position.x)
        #                 self.v.append(pose.position.y)
        #                 self.area.append(pose.position.z)
        #                 self.yaw.append(pose.orientation.y)
        #                 self.m.append(pose.orientation.z * -1) # Tirar sinal de negativo

        # else:
        #     if self.teste == 1:
        #         self.ajeitaValores(var=0)
        #         print(self.target[0])
        #         self.teste = 2
                
        # if self.VARREDURA == 1 or self.VARREDURA == 3:
        #     for pose in target.poses:
        #         if pose.orientation.w == self.armazenar:
        #             if pose.orientation.z < 7:
        #                 self.u.append(pose.position.x)
        #                 self.v.append(pose.position.y)
        #                 self.area.append(pose.position.z)
        #                 self.yaw.append(pose.orientation.y)
        #                 self.m.append(pose.orientation.z)
        if self.startSM == 1 and (self.time() - self.tt > 22):  #ALTERAR URGENTE..SOLUCAO TEMPORARIA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

            if self.ROTINA_DIEGO == 20:
                print("ignorando o diego")
                if self.uniqueBaseBaixo == 0:
                    self.timeBaseBaixo = self.time()
                    self.uniqueBaseBaixo = 1

                if (self.time() - self.timeBaseBaixo > self.tempoBaseBaixo):
                    #self.ROTINA_DIEGO = 21
                    self.uniqueBaseBaixo = 0

                    self.ROTINA_DIEGO = 0
                    
            else:
                for pose in target.poses:
                    self.u.append(pose.position.x)
                    self.v.append(pose.position.y)
                    self.area.append(pose.position.z)
                    self.yaw.append(pose.orientation.y)
                    self.m.append(pose.orientation.z)
                    self.flag.append(pose.orientation.w)

                a = np.where(np.asarray(self.flag) == self.baseToda)
                b = np.where(np.asarray(self.flag) == self.parteBaixo)

                if len(a[0]) != 0:
                    self.ROTINA_DIEGO = 50
                    print("PEGUEI BASE TODAAAAAA!!!!!!!!!!!!")
                elif len(b[0]) != 0:
                    
                    # Se ele perder nao vai ser continuo

                    if self.uniqueBaseBaixo == 0:
                        self.ROTINA_DIEGO = 60
                        self.u = []
                        self.v = []
                        self.area = []
                        self.yaw = []
                        self.m = []
                        self.flag = []
                        self.timeBaseBaixo = 0
                        self.uniqueBaseBaixo = 1
                        print("rotina de aproximcao")
                        self.goToLocal(flag=self.abort)
                        print("rotina de aproximcao")

                    if self.timeBaseBaixo < 50:
                        self.timeBaseBaixo += 1
                        for pose in target.poses:
                            self.u.append(pose.position.x)
                            self.v.append(pose.position.y)
                            self.area.append(pose.position.z)
                            self.yaw.append(pose.orientation.y)
                            self.m.append(pose.orientation.z)
                            self.flag.append(pose.orientation.w)
                
    def callbackPosicao(self, data):
        orbs = (0b11000000 & data.data) >> 6
        status = (0b00001111 & data.data)
        if(self.statusNovo != status):
            print("STATUS:" + str(status))
            self.statusNovo = status
        # print(orbs)

        # Como entra no perdeu orbs?
        if orbs == self.perdeu: #and (self.ROTINA_DIEGO != 5 and self.ROTINA_DIEGO != 3):
            self.PERDEU_ORBS = 1

        # ---------------------------- VARREDURA ---------------------------------------------
        if self.VARREDURA >= 1:
            if status == self.idle:
                if (self.time() - self.timeStop > self.tempoParar):
                    if self.VARREDURA == 1:
                        self.timeVarredura = self.time()
                        self.VARREDURA = 2
                        print("esperando varredura")
                    
                    if (self.time() - self.timeVarredura > self.tempoVarredura):
                        if self.VARREDURA == 2 and self.estado != 9:
                            self.ajeitaValores(var=0)
                            self.rotacionar(-math.pi/2)
                            self.VARREDURA = 3
                            self.estado = 9

                        elif self.VARREDURA == 3 and self.estado != 9:
                            self.timeVarredura = self.time()
                            self.VARREDURA = 4
                            print("esperando varredura")

                        elif self.VARREDURA == 4:
                            twist = Twist()
                            twist.angular.y = 0

                            self.myCamera.publish(twist)
                            self.timeVarredura = self.time()
                            self.VARREDURA = 5
                            print("camera em 0")
                            
                        elif self.VARREDURA == 5:
                            self.ajeitaValores(var=1)
                            self.rotacionar(math.pi/2)
                            self.VARREDURA = 6
                            self.estado = 9

                        elif self.VARREDURA == 6 and self.estado != 9:
                            self.mesclarBases()
                            self.ROTINA_ORBS = 1
                            self.VARREDURA = 0

            elif status == self.cheguei:
                self.goToGlobal(flag=self.idle)
                self.timeStop = self.time()
                self.estado = 10

            elif status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")

        # ---------------------------- PERDEU ORBS SLAM ---------------------------------------
        if self.PERDEU_ORBS >= 1 and self.startSM == 1:
            if self.timePorbs == 0:
                self.timePorbs = self.time()

            if self.time() - self.timePorbs < self.tempoPOrbs:
                if self.PERDEU_ORBS == 1:
                    self.goToGlobal(flag=self.abort)
                    self.PERDEU_ORBS = 2

                elif status == self.abortei:
                    self.timeStop = self.time()
                    self.goToGlobal(flag=self.idle)
                    self.PERDEU_ORBS = 3

                elif status == self.idle:
                    if (self.time() - self.timeStop > self.tempoParar):
                        if self.estado == 9:
                            if self.PERDEU_ORBS == 3:
                                self.timeRecOrb = self.time()
                                print("esperando orbs voltar")
                                if self.time() - self.timeRecOrb > self.tempoRecOrb:
                                    self.goToLocal(x=0.2)
                                    self.PERDEU_ORBS = 4
                                    self.estado = 8
                            if self.PERDEU_ORBS == 4:
                                self.goToLocal(x=-0.2)
                                self.PERDEU_ORBS = 5
                                self.estado = 8
                            if self.PERDEU_ORBS == 5:
                                self.PERDEU_ORBS = 3

                elif status == self.busy:
                    if self.estado != 2:
                        self.estado = 2
                        print("ocupado")
                
                elif status == self.cheguei:
                    self.estado = 9
                    self.goToGlobal(flag=self.idle)
                    self.timeStop = self.time()

                if orbs == self.comEscala:
                    self.PERDEU_ORBS = 0
            else:
                self.PERDEU_ORBS = 0
                self.usingOrbs = 0
                # mandar flag pro joao matar o orbs
                # checar se ele me retorno algo disso
                # inicia a nova rotina


        # -------------------------- ORB SLAM -------------------------------------------------
        if self.ROTINA_ORBS >= 1 and self.startSM == 1 and self.PERDEU_ORBS == 0:
            if status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")

            elif status == self.cheguei:
                self.estado = 9
                self.goToGlobal(flag=self.idle)
                self.timeStop = self.time()

            elif status == self.idle:
                if (self.time() - self.timeStop > self.tempoParar):
                    if self.estado == 9:
                        if self.ROTINA_ORBS == 1:
                            print("ligou orbs")
                            self.iniciaOrb.publish(0xFF)
                            self.ROTINA_ORBS = 4

                        elif self.ROTINA_ORBS == 4: # Vai pra direita
                            print("iniciou quadrado")
                            self.ROTINA_ORBS = 20 
                            self.estado = 7

                        elif self.ROTINA_ORBS == 30:  # vai pra frente
                            self.ROTINA_ORBS = 40
                            self.estado = 7

                        elif self.ROTINA_ORBS == 50: # vai pra esquerda
                            self.ROTINA_ORBS = 60
                            self.estado = 7

                        elif self.ROTINA_ORBS == 70: # vai pra tras
                            self.ROTINA_ORBS = 80
                            self.estado = 7

                        elif self.ROTINA_ORBS == 90: # vai pra tras
                            self.iniciaLog.publish(0XFF)
                            self.ROTINA_ORBS = 100
                            self.estado = 7
                    
                    if self.estado == 7:
                        if self.ROTINA_ORBS == 20:
                            print("direita")
                            self.goToLocal(y=0.3 * self.direcaoOrbs * -1, t=3)
                            self.ROTINA_ORBS = 30
                            self.estado = 8

                        elif self.ROTINA_ORBS == 40:
                            print("frente")
                            self.goToLocal(x=0.3, t=3)
                            self.ROTINA_ORBS = 50
                            self.estado = 8

                        elif self.ROTINA_ORBS == 60:
                            print("esquerda")
                            self.goToLocal(y=0.3 * self.direcaoOrbs, t=3)
                            self.ROTINA_ORBS = 70
                            self.estado = 8

                        elif self.ROTINA_ORBS == 80:
                            print("tras")
                            self.goToLocal(x=-0.3, t=3)
                            self.ROTINA_ORBS = 90
                            self.estado = 8

                        elif self.ROTINA_ORBS == 100:
                            print("frente")
                            self.goToLocal(x=0.3, t=3)
                            self.ROTINA_ORBS = 13
                            self.estado = 8

                    if orbs == self.comEscala:
                        self.estado = 6
                        self.goToLocal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                        self.pos += 1
                        self.ROTACIONAR = 0
                        self.ROTINA_ORBS = 0

        # ------------------------------------- ROTINA DIEGO --------------------------------------
        if self.startSM == 1 and self.ROTINA_DIEGO >= 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS) == 0:
            # if self.ROTINA_DIEGO == 50: # Pousar
            #     self.goToLocal(self.abort)
            # elif self.ROTINA_DIEGO == 60:
            #     if (self.time() - self.timeBaseBaixo > self.tempoBaseBaixo):
            #         print("aproxima")
            #         if abs(self.curr_pos["x"] - 3.5) <= 0.4 and abs(self.curr_pos["y"] - 0.5) <= 0.4:
            #             sub = 1.5
            #         elif abs(self.curr_pos["x"] - 0.5) <= 0.4 and abs(self.curr_pos["y"] - 7) <= 0.4:
            #             sub = 1
            #         else:
            #             sub = 0
            #         self.targetPixel["x"] = np.mean(self.u) * (self.curr_pos["z"] - sub) / self.focoCamera["x"]
            #         self.targetPixel["y"] = np.mean(self.v) * (self.curr_pos["z"] - sub) / self.focoCamera["y"]

            #         self.goToLocal(self.targetPixel["x"], self.targetPixel["y"])
            #         self.ROTINA_DIEGO = 100


            if status == self.abortei:
                self.goToGlobal(flag=self.idle)
                print("foi abortado na rotina diego")

            elif status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")

            elif status == self.cheguei:
                self.estado = 6
                print("cheguei")
                if self.ROTINA_DIEGO == 100:
                    self.ROTINA_DIEGO = 50
                    print("vai pousar")
                self.goToLocal(flag=self.idle)
                self.timeStop = self.time()

            elif status == self.idle:
                if self.ROTINA_DIEGO == 50:
                    #if self.checarAlvo([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]]):
                    self.ROTINA_DIEGO = 20
                    print("pousando")
                    self.desce.publish(Empty())
                    print("pousou")
                    self.dormir()
                    self.sobe.publish(Empty())
                    self.dormir()
                    self.goToLocal(z=self.alturaDrone - self.curr_pos["z"])
                    self.pos -= 1
                    self.ROTACIONAR = 0
                    


                    

                        # self.alvosBloqueados.append([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]])

                        # self.sobe.publish(Empty())
                        # self.dormir(t=6)
                        # self.estado = 200
                    #self.ROTINA_DIEGO = 20

                # if self.ROTINA_DIEGO == 20 or self.ROTINA_DIEGO == 21:
                #     if self.estado == 200:
                #         self.goToGlobal(z=self.alturaDrone)
                #         self.estado = 201
                #     else:
                #         self.pos += 1 
                #         self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
                #         if self.ROTINA_DIEGO == 20:
                #             self.ROTINA_DIEGO == 21
                #         else:
                #             self.ROTINA_DIEGO = 0

                if self.ROTINA_DIEGO == 60:
                    if self.timeBaseBaixo == 50:
                        self.timeBaseBaixo += 1
                        print("tme: " + str(self.timeBaseBaixo))
                        # if abs(self.curr_pos["x"] - 3.5) <= 0.4 and abs(self.curr_pos["y"] - 0.5) <= 0.4:
                        #     sub = 1.5
                        # elif abs(self.curr_pos["x"] - 0.5) <= 0.4 and abs(self.curr_pos["y"] - 7) <= 0.4:
                        #     sub = 1
                        # else:
                        sub = 0
                        # self.targetPixel["x"] = np.mean(self.u) * (self.curr_pos["z"] - sub) / self.focoCamera["x"]
                        # self.targetPixel["y"] = np.mean(self.v) * (self.curr_pos["z"] - sub) / self.focoCamera["y"]
                        self.targetPixel["x"] = np.mean(self.u) * (self.curr_pos["z"] - sub) / self.focoCamera["x"]
                        self.targetPixel["y"] = np.mean(self.v) * (self.curr_pos["z"] - sub) / self.focoCamera["y"]
                        print("esta aproximando")
                        self.estado = 6
                        self.goToLocal(self.targetPixel["x"], self.targetPixel["y"])
                        self.ROTINA_DIEGO = 100

        # ------------------------------------- ROTINA NORMAL ------------------------------------------------------
        
        if self.startSM == 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS) == 0 and (self.ROTINA_DIEGO == 20 or self.ROTINA_DIEGO == 0):
            if status == self.idle: # Idle
                if self.estado == 0:
                    if(self.takeOffBusy == 0):
                        self.sobe.publish(Empty())
                        self.takeOffBusy = 1
                        self.timeTakeOff = self.time()
                        print("subindo - takeoff")
                    else:
                        if(self.time() - self.timeTakeOff > self.tempoTakeOff):
                            self.estado = 1
                elif self.estado == 1:
                    #self.goToGlobal(0, 0, 2)
                    self.goToLocal(z = self.alturaDrone - self.curr_pos["z"])
                    #self.desce.publish(Empty())
                    #self.goToLocal(0,0,2)
                    # self.goToGlobal(self.rotas["x"][0], self.rotas["y"][0], self.rotas["z"][0], self.rotas["yaw"][0])
                    self.estado = 20
                elif self.estado == 3 or self.estado == 13:
                    if(self.time() - self.timeStop > self.tempoParar):
                        self.stopBusy = 0  
                        if self.ROTACIONAR == 0: # Caso tenha chego em um waypoint mas ainda nao rotacionou
                            # Pouso Forcado
                            if self.rotas["x"][self.pos] == -90:
                                print("pousando")
                                self.desce.publish(Empty())
                                self.startSM = 0
                            else:
                                
                                self.ROTACIONAR = 2
                        elif self.ROTACIONAR == 1: # Caso tenha chegado em um waypoint e a rotacao esta correta
                            #self.goToLocal(z=1.5)
                            #self.goToGlobal(x=self.curr_pos["x"],y=self.curr_pos["y"],z=self.rotas["z"][self.pos])
                            self.ROTACIONAR = 2 
                            #self.estado = 10
                        elif self.ROTACIONAR == 2:
                            self.goToLocal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                            self.pos += 1
                            self.ROTACIONAR = 0
                            self.estado = 10
                
                if self.estado == 5:
                    print("pousando")
                    self.desce.publish(Empty()) 
    
            elif status == self.cheguei: # Cheguei
                if self.stopBusy == 0:
                    print("cheguei")
                    self.goToGlobal(flag=self.idle)
                    print("Foi para o idle no cheguei")
                    self.stopBusy = 1
                    self.timeStop = self.time()
                    if self.estado == 2:
                        self.estado = 3

            elif status == self.busy: # Ocupado
                if self.estado == 9:
                    self.ROTACIONAR = 1
                if self.estado == 8:
                    self.VARREDURA = 1
                if self.estado != 2:
                   

                    print("estou ocupado")
                    if self.estado != 5:
                        self.estado = 2
            
def main():
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
