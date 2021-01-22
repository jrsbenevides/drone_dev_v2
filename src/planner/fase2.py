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
        self.alturaDrone = 2
        self.pontoAtual = {"x": 0, "y": 0, "z": 0}
        self.alvosBloqueados = []
        self.rotas = {"x": [0, 0.75, 7, 0, -90], "y": [0, 2, 7, 0, -90], "z": [self.alturaDrone, self.alturaDrone, self.alturaDrone, -90], "yaw": [0, 0, 0, 0]}
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
        self.anguloDobra = 20

        self.startSM = 0
        self.takeOffBusy = 0
        self.estado = 0
        self.stopBusy = 0
        self.timeStop = 0
        self.timeTakeOff = 0
        self.timeRecOrb = 0
        self.timeVarredura = 0
        self.timeBaseBaixo = 0
        self.ROTINA_ORBS = 0
        self.PERDEU_ORBS = 0
        self.usingOrbs = 0
        self.uniqueBaseBaixo = 0

        self.tempoParar = 1
        self.tempoTakeOff = 6
        self.tempoRecOrb = 5
        self.tempoVarredura = 5
        self.tempoBaseBaixo = 5

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
        self.startSM = 1
        print("comecou")

    def dormir(self, t=5):
        rospy.sleep(t)

    def time(self):
        return rospy.get_rostime().secs + (rospy.get_rostime().nsecs/1e9)

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
        
        if flag != self.desFinal or flag != self.calibraAlvo:
            r = [[np.cos(angle), np.sin(angle) * -1, 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
            pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x, y, z]))

        pose = PoseStamped()
        pose.pose.position.x = pos_local[0]
        pose.pose.position.y = pos_local[1]
        pose.pose.position.z = pos_local[2]
        if yaw == 0:
            pose.pose.orientation.x = angle - self.curr_pos["yaw"]
        else:
            pose.pose.orientation.x = yaw
        pose.pose.orientation.y = self.deftempo(x, y, z) if t == 0 else t
        pose.pose.orientation.z = flag

        if self.ROTINA_DIEGO != 5 and flag == self.vai:
            print("Indo para o ponto: " + str(pos_local))
            print("tempo: " + str(pose.pose.orientation.y))

        self.cmd.publish(pose)

    def goToGlobal(self, x=100, y=100, z=100, yaw=100, t=0, flag=10):
        x1 = self.curr_pos["x"] if x == 100 else x - self.curr_pos["x"] - 1
        y1 = self.curr_pos["y"] if y == 100 else y - self.curr_pos["y"] - 0.75
        z1 = self.curr_pos["z"] if z == 100 else z - self.curr_pos["z"] - 0.5
        psi0 = self.curr_pos["yaw"] if yaw == 100 else yaw - self.curr_pos["yaw"]

        self.goToLocal(x1, y1, z1, psi0, t, flag)

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

    def callbackVisao(self, target):
        # u - v - area - pitch - yaw - m - flag
        # quando for mandar em pixel checar se o y ta confiavel pelo orbs
                
        if self.VARREDURA == 0:
            if self.ROTINA_DIEGO == 20:
                if self.uniqueBaseBaixo == 0:
                    self.timeBaseBaixo = self.time()
                    self.uniqueBaseBaixo = 1

                if (self.time() - self.timeBaseBaixo > self.tempoBaseBaixo):
                    self.ROTINA_DIEGO = 21
                    self.uniqueBaseBaixo = 0
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

                if len(a) > 0:
                    self.ROTINA_DIEGO = 50
                elif len(b) > 0:
                    self.ROTINA_DIEGO = 60
                    if self.uniqueBaseBaixo == 0:
                        self.u = []
                        self.v = []
                        self.area = []
                        self.yaw = []
                        self.m = []
                        self.flag = []
                        self.timeBaseBaixo = self.time()
                        self.uniqueBaseBaixo = 1

                    if (self.time() - self.timeBaseBaixo < self.tempoBaseBaixo):
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

        # Como entra no perdeu orbs?
        if orbs == self.perdeu: #and (self.ROTINA_DIEGO != 5 and self.ROTINA_DIEGO != 3):
            self.PERDEU_ORBS = 1

        # ---------------------------- PERDEU ORBS SLAM ---------------------------------------
        if self.PERDEU_ORBS >= 1 and self.startSM == 1:
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
                        self.goToLocal(x=self.rotas["x"][1], y=self.rotas["y"][1])
                        self.pos += 1
                        self.ROTACIONAR = 0
                        self.ROTINA_ORBS = 0

        # ------------------------------------- ROTINA DIEGO --------------------------------------
        if self.startSM == 1 and self.ROTINA_DIEGO >= 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS) == 0:
            if self.ROTINA_DIEGO == 50: # Pousar
                self.goToLocal(self.abort)
            elif self.ROTINA_DIEGO == 60:
                if (self.time() - self.timeBaseBaixo > self.tempoBaseBaixo):
                    if abs(self.curr_pos["x"] - 3.5) <= 0.4 and abs(self.curr_pos["y"] - 0.5) <= 0.4:
                        sub = 1.5
                    elif abs(self.curr_pos["x"] - 0.5) <= 0.4 and abs(self.curr_pos["y"] - 7) <= 0.4:
                        sub = 1
                    else:
                        sub = 0
                    self.targetPixel["x"] = np.mean(self.u) * (self.curr_pos["z"] - sub) / self.focoCamera["x"]
                    self.targetPixel["y"] = np.mean(self.v) * (self.curr_pos["z"] - sub) / self.focoCamera["y"]

                    self.goToLocal(self.targetPixel["x"], self.targetPixel["y"])
                    self.ROTINA_DIEGO = 0

            if status == self.abortei:
                self.estado = 50
                self.goToGlobal(flag=self.idle)
                print("foi abortado")

            elif status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")

            elif status == self.cheguei:
                self.estado = 6
                self.goToGlobal(flag=self.idle)
                self.timeStop = self.time()

            elif status == self.idle:
                if self.estado == 50:
                    if self.checarAlvo([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]]):
                        self.desce.publish(Empty())
                        self.dormir(t=6)

                        self.alvosBloqueados.append([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]])

                        self.sobe.publish(Empty())
                        self.dormir(t=6)
                        self.estado = 200
                    self.ROTINA_DIEGO = 20

                if self.ROTINA_DIEGO == 20 or self.ROTINA_DIEGO == 21:
                    if self.estado == 200:
                        self.goToGlobal(z=self.alturaDrone)
                        self.estado = 201
                    else:
                        self.pos += 1 
                        self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
                        if self.ROTINA_DIEGO == 20:
                            self.ROTINA_DIEGO == 21
                        else:
                            self.ROTINA_DIEGO = 0

        # ------------------------------------- ROTINA NORMAL ------------------------------------------------------
        
        if self.startSM == 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS + self.ROTINA_DIEGO) == 0:
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
                    self.goToGlobal(self.rotas["x"][0], self.rotas["y"][0], self.rotas["z"][0], self.rotas["yaw"][0])
                    #self.goToLocal(0,0,2)
                    # self.goToGlobal(self.rotas["x"][0], self.rotas["y"][0], self.rotas["z"][0], self.rotas["yaw"][0])
                    self.estado = 8
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
                                self.rotacionar(angle=self.toRadian(self.anguloDobra))
                                self.estado = 4
                                self.ROTACIONAR = 1
                        elif self.ROTACIONAR == 1: # Caso tenha chegado em um waypoint e a rotacao esta correta
                            #self.goToLocal(z=1.5)
                            self.goToGlobal(x=self.rotas["x"][2], y=self.rotas["y"][2])
                            self.ROTACIONAR = 2 
                            self.estado = 10
                        elif self.ROTACIONAR == 2:
                            self.goToGlobal(x=0, y=0)
                            self.pos += 1
                            self.ROTACIONAR = 0
                            self.estado = 5
                    
                if self.estado == 5:
                    print("pousando")
                    self.desce.publish(Empty())
                    self.startSM = 0 
    
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
                    self.ROTINA_ORBS = 1
                if self.estado != 2:
                    if self.estado != 5:
                        print("estou ocupado")
                        self.estado = 2
            
def main():
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
