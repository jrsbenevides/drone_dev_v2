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
        self.rotas = {"x": [0, 3.5, -90], "y": [0, 0.5, -90], "z": [3, 3, -90], "yaw": [0, 0, 0]}
        self.pos = 1
        self.ROTINA_DIEGO = 0
        self.LIMIAR_PRECISAO_ALVO = 0.8
        #self.erroAlvo = 0.2
        self.focoCamera = {"x": 595.035494, "y": 594.674498} 
        self.targetPixel = {"x": [], "y": []}
        self.alvo1 = {"x": [], "y": [], "z": [], "yaw": []}
        self.alvo2 = {"x": [], "y": [], "z": [], "yaw": []}
        self.alvo3 = {"x": [], "y": [], "z": [], "yaw": []}
        self.alvo4 = {"x": [], "y": [], "z": [], "yaw": []}
        self.alvo = {"local": {"x": 0, "y": 0, "z": 0, "yaw": 0}, "global": {"x": 2.5, "y": 0.75, "z": 2}}
        #self.guardaLocal = {"x": [], "y": [], "z": [], "yaw": []}
        self.ROTACIONAR = 1
        self.ajusteTempo = 5
        self.viconOne = 0
        self.pose_home = {"x": 1, "y": 0.75, "z": 0.5}
        self.psi, self.po, self.r = 0, 0, 0
        self.direcaoOrbs = 1 # 1 = esquerda e -1 = direita

        self.u = []
        self.v = []
        self.Xr = []
        self.Yr = []
        self.Zr = []
        self.Xmean = 0
        self.Ymean = 0
        self.Zmean = 0        

        self.startSM = 0
        self.takeOffBusy = 0
        self.estado = 0
        self.stopBusy = 0
        self.timeStop = 0
        self.timeTakeOff = 0
        self.timeRecOrb = 0
        self.timeVarredura = 0
        self.ROTINA_ORBS = 0
        self.PERDEU_ORBS = 0
        self.VARREDURA = 0

        self.tempoParar = 1
        self.tempoTakeOff = 6
        self.alturaDrone = 3
        self.tempoRecOrb = 5
        self.tempoVarredura = 5
        self.target = {"x":0,"y":0}

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

        self.cmd = rospy.Publisher("planejamento", PoseStamped, queue_size=10)
        self.iniciaOrb = rospy.Publisher("iniciaOrbSlam", UInt8, queue_size=10)
        self.iniciaLog = rospy.Publisher("iniciaLog", UInt8, queue_size=10)
        self.myCamera   = rospy.Publisher("/bebop/camera_control", Twist, queue_size = 1)
        self.statusPlanning_sub = rospy.Subscriber("statusPlanning", UInt8, self.callbackPosicao)
        self.perceptionTarget = rospy.Subscriber("/perception/target", PoseArray, self.callbackVisao)
        #self.perceptionOdom = rospy.Subscriber("/bebop/odom", Odometry, self.callbackOdom)
        #self.perceptionTarget = rospy.Subscriber("/bebop/odom", Odometry, self.callbackOdom)
        #self.perceptionTarget = rospy.Subscriber("/scale/log", PoseStamped, self.callbackLog)
        self.sobe = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.desce = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.vicon_sub = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, self.callbackVicon)
    
        rospy.init_node('Global_Planner', anonymous=True)

        self.dormir(t=1)
        twist = Twist()
        twist.angular.y = 0
        self.myCamera.publish(twist)
        self.dormir(t=1)
        self.startSM = 1
        print("comecou")

    def dormir(self, t=5):
        rospy.sleep(t)

    def toRadian(self, grau):
        return grau * math.pi / 180

    def toGrau(self, radiano):
        return radiano * 180 / math.pi

    def deftempo(self, objx, objy, objz):
        euc = math.sqrt(pow(objx, 2) + pow(objy, 2) + pow(objz, 2))

        return euc * self.ajusteTempo

    def callbackOdom(self, odom):
    	self.curr_pos["z"] = odom.pose.position.z

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
        angle = self.toRadian(self.ROTACAO_ATUAL) 
        
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

    def goToGlobal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
    	print("Ponto Global: " + str(x) + " " + str(y) + " "+ str(z))
        x1 = x - self.curr_pos["x"]
        y1 = y - self.curr_pos["y"]
        z1 = z - self.curr_pos["z"]
        psi0 = yaw - self.curr_pos["yaw"]

        self.goToLocal(x1, y1, z1, psi0, t, flag)

    #def defangle(objx, objy, objz):
    #    angle = math.atan2(objx - self.curr_pos.x, objy - self.curr_pos.y)
    # 
    #    return angle

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

    def callbackVicon(self, vicon):
        pv = vicon.transform.translation
        ov = vicon.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([ov.x, ov.y, ov.z, ov.w])        
        if self.viconOne == 0:
            self.psi0 = yaw
            self.po = pv
            self.viconOne = 1
        else:
            self.r = [[np.cos(self.psi0), np.sin(self.psi0) * -1, 0], [np.sin(self.psi0), np.cos(self.psi0), 0], [0, 0, 1]]
            posicao = np.dot(np.transpose(np.asarray(self.r)), np.asarray([[pv.x - self.po.x], [pv.y - self.po.y], [pv.z - self.po.z]]))
            self.curr_pos["x"] = posicao[0]
            self.curr_pos["y"] = posicao[1]
            self.curr_pos["z"] = posicao[2]
            self.curr_pos["yaw"] = yaw - self.psi0

            # print("Xr: " + str(self.curr_pos["x"]))
            # print("Yr: " + str(self.curr_pos["y"]))
            # print("Zr: " + str(self.curr_pos["z"]))

    # def callbackLog(self, log):
    # 	self.curr_pos["x"] = log.pose.position.x
    # 	self.curr_pos["y"] = log.pose.position.y
    # 	self.curr_pos["z"] = log.pose.position.z
    # 	roll, pitch, yaw = euler_from_quaternion([log.pose.orientation.x, log.pose.orientation.y, log.pose.orientation.z, log.pose.orientation.w]) 
    # 	self.curr_pos["yaw"] = yaw

    # def callbackOdom(self, odom):
    #     self.curr_pos["z"] = odom.pose.position.z

    def goToHome(self):
        # vai em que angulo para a home?
        self.goToGlobal(self.pose_home["x"], self.pose_home["y"], self.pose_home["z"])
        self.estado = 5

    def checkRange(self, media, valor, offset=10000): # Falta calibrar offset
        if media == 0:
            return True
        else:
            if media - offset < valor and media + offset > valor:
                return True
            else:
                return False

    def callbackVisao(self, target):
        # u - v - area - pitch - yaw - m - flag]
    
        if len(self.u) == 50:
            self.targetPixel["x"] = np.mean(self.u)
            self.targetPixel["y"] = np.mean(self.v)
            self.Xmean = np.mean(self.Xr)
            self.Ymean = np.mean(self.Yr)
            self.Zmean = np.mean(self.Zr)

            print("Xr: " + str(self.Xmean))
            print("Yr: " + str(self.Ymean))
            print("H: "  + str(self.Zmean))
            print("dx: " + str(np.mean(self.targetPixel["x"])))
            print("dy: " + str(np.mean(self.targetPixel["y"])))
            # self.goToLocal(x=np.mean(self.targetPixel["x"]), y=np.mean(self.targetPixel["y"]),t=4)
            self.u.append(1)

        else:
            for i in range(len(target.poses)):
                self.u.append(target.poses[i].position.x)
                self.v.append(target.poses[i].position.y)
                self.Xr.append(self.curr_pos["x"])
                self.Yr.append(self.curr_pos["y"])
                self.Zr.append(self.curr_pos["z"])

        # if len(self.u) == 50:
        #     self.targetPixel["x"] = np.mean(self.u) * 2 / self.focoCamera["x"]
        #     self.targetPixel["y"] = np.mean(self.v) * 2 / self.focoCamera["y"]

        #     print("x: " + str(np.mean(self.targetPixel["x"])))
        #     print("y: " + str(np.mean(self.targetPixel["y"])))

        #     self.goToLocal(x=np.mean(self.targetPixel["x"]), y=np.mean(self.targetPixel["y"]),t=4)
        #     self.u.append(1)

        # else:
        #     for i in range(len(target.poses)):
        #         self.u.append(target.poses[i].position.x)
        #         self.v.append(target.poses[i].position.y)                


        
        

        # u = []
        # v = []
        # area = []
        # yaw = []
        # m = []
        # flag = []
        
        # for i in range(len(target.pose)):
        #         u.append(target.poses[i].position.x)
        #         v.append(target.poses[i].position.y)
        #         area.append(target.poses[i].position.z)
        #         yaw.append(target.poses[i].orientation.y)
        #         m.append(target.poses[i].orientation.z)
        #         flag.append(target.poses[i].orientation.w)

        # newyaw = {"b1": [], "b2": [], "b3": [], "b4": [], "b5": []}
        # newm = {"b1": [], "b2": [], "b3": [], "b4": [], "b5": []}

        # if self.VARREDURA >= 1:
        #     index = np.where(np.asarray(flag) == self.armazenar)
        #     for i in index: 
        #         # yaw 0 and camera -50
        #         # yaw -90 and camera -50
        #         # yaw -90 and camera 0
        #         # yaw 0 and camera 0
        #         if self.VARREDURA == 1 or self.VARREDURA == 3 or self.VARREDURA == 5 or self.VARREDURA == 7: 
        #             if self.checkRange(np.mean(newm["b1"]), m[i]):
        #                 newyaw["b1"].append(yaw[i])
        #                 newm["b1"].append(m[i])
        #             elif self.checkRange(np.mean(newm["b2"]), m[i]):
        #                 newyaw["b2"].append(yaw[i])
        #                 newm["b2"].append(m[i])
        #             elif self.checkRange(np.mean(newm["b3"]), m[i]):
        #                 newyaw["b3"].append(yaw[i])
        #                 newm["b3"].append(m[i])
        #             elif self.checkRange(np.mean(newm["b4"]), m[i]):
        #                 newyaw["b4"].append(yaw[i])
        #                 newm["b4"].append(m[i])
        #             elif self.checkRange(np.mean(newm["b5"]), m[i]):
        #                 newyaw["b5"].append(yaw[i])
        #                 newm["b5"].append(m[i])
    
        #         # elif self.VARREDURA == 2 or self.VARREDURA == 4 or self.VARREDURA == 6 or self.VARREDURA == 8:
        #         #     for q in range(5):
        #         #         if np.mean(newm["b"+str(q+1)]) > 0:
        #         #             for j in newm["b"+str(q+1)]:
        #         #                 if self.VARREDURA == 2:
        #         #                     self.alvo1["x"].append(np.mean(newm["b"+str(q+1)]) * np.cos(np.mean(yaw["b"+str(q+1)])))
        #         #                     self.alvo1["y"].append(np.mean(newm["b"+str(q+1)]) * np.sin(np.mean(yaw["b"+str(q+1)])))
        #         #                 elif self.VARREDURA == 4:
        #         #                     self.alvo2["x"].append(np.mean(newm["b"+str(q+1)]) * np.cos(np.mean(yaw["b"+str(q+1)])))
        #         #                     self.alvo2["y"].append(np.mean(newm["b"+str(q+1)]) * np.sin(np.mean(yaw["b"+str(q+1)])))
        #         #                 elif self.VARREDURA == 6:
        #         #                     self.alvo3["x"].append(np.mean(newm["b"+str(q+1)]) * np.cos(np.mean(yaw["b"+str(q+1)])))
        #         #                     self.alvo3["y"].append(np.mean(newm["b"+str(q+1)]) * np.sin(np.mean(yaw["b"+str(q+1)])))
        #         #                 elif self.VARREDURA == 8:
        #         #                     self.alvo4["x"].append(np.mean(newm["b"+str(q+1)]) * np.cos(np.mean(yaw["b"+str(q+1)])))
        #         #                     self.alvo4["y"].append(np.mean(newm["b"+str(q+1)]) * np.sin(np.mean(yaw["b"+str(q+1)])))
                        
        #             newyaw = {"b1": [], "b2": [], "b3": [], "b4": [], "b5": []}
        #             newm = {"b1": [], "b2": [], "b3": [], "b4": [], "b5": []}
                 
        #         elif self.VARREDURA == 8:
        #             a = [[[],[]], [[],[]], [[],[]], [[],[]], [[],[]]]
        #             b = [[[],[]], [[],[]], [[],[]], [[],[]], [[],[]]]
        #             c = [[[],[]], [[],[]], [[],[]], [[],[]], [[],[]]]
        #             d = [[[],[]], [[],[]], [[],[]], [[],[]], [[],[]]]

        #             for q in range(5):
        #                 if np.mean(newm["b"+str(q+1)]) > 0:
        #                     a[q][0] = abs(self.alvo1["x"][q]) + self.curr_pos["x"]
        #                     a[q][1] = abs(self.alvo1["y"][q]) + self.curr_pos["y"]

        #                     b[q][0] = abs(self.alvo2["x"][q]) + self.curr_pos["y"]
        #                     b[q][1] = abs(self.alvo2["y"][q]) + self.curr_pos["x"]
        #                     c[q][0] = abs(self.alvo3["x"][q]) + self.curr_pos["y"]
        #                     c[q][1] = abs(self.alvo3["y"][q]) + self.curr_pos["x"]
                            
        #                     d[q][0] = abs(self.alvo4["x"][q]) + self.curr_pos["x"]
        #                     d[q][1] = abs(self.alvo4["y"][q]) + self.curr_pos["y"]

        #             # Cruzar alvos para definir o final
                
        # else:
        #     if np.where(np.asarray(flag) == self.baseToda):
        #         self.ROTINA_DIEGO == 50

        #     elif np.where(np.asarray(flag) == self.parteBaixo):
        #         l = np.where(np.asarray(flag) == self.parteBaixo)
        #         for f in l:
        #             self.targetPixel["x"].append(u[f] * self.curr_pos["z"] / self.focoCamera["x"])
        #             self.targetPixel["y"].append(v[f] * self.curr_pos["z"] / self.focoCamera["y"])
        #         self.ROTINA_DIEGO = 60
            
    def callbackPosicao(self, data):
        orbs = (0b11000000 & data.data) >> 6
        status = (0b00001111 & data.data)
        # if status == self.cheguei:
        # 	self.goToLocal(flag=self.idle)
        # 	self.estado = 5
            
        # # if orbs == self.perdeu and (self.ROTINA_DIEGO != 5 and self.ROTINA_DIEGO != 3):
        # #     self.PERDEU_ORBS = 1

        # if status == self.idle:
        # 	if self.estado == 5:
        # 		self.desce.publish(Empty())
        # # ---------------------------- VARREDURA ---------------------------------------------
        # if self.VARREDURA >= 1:
        #     if status == self.idle:
        #         if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
        #             if self.VARREDURA == 1:
        #                 self.timeVarredura = rospy.get_rostime().secs
        #                 self.VARREDURA = 2
        #                 print("esperando varredura")
                    
        #             if (rospy.get_rostime().secs - self.timeVarredura > self.tempoVarredura):
        #                 if self.VARREDURA == 2 and self.estado != 9:
        #                     self.rotacionar(-math.pi/2)
        #                     self.VARREDURA = 3
        #                     self.estado = 9

        #                 elif self.VARREDURA == 3 and self.estado != 9:
        #                     self.timeVarredura = rospy.get_rostime().secs
        #                     self.VARREDURA = 4
        #                     print("esperando varredura")

        #                 elif self.VARREDURA == 4:
        #                     twist = Twist()
        #                     twist.angular.y = 0

        #                     self.myCamera.publish(twist)
        #                     self.timeVarredura = rospy.get_rostime().secs
        #                     self.VARREDURA = 5
        #                     print("camera em 0")
                            
        #                 elif self.VARREDURA == 5:
        #                     self.timeVarredura = rospy.get_rostime().secs
        #                     self.VARREDURA = 6
        #                     print("esperando varredura")

        #                 elif self.VARREDURA == 6:
        #                     self.rotacionar(math.pi/2)
        #                     self.VARREDURA = 7
        #                     self.estado = 9

        #                 elif self.VARREDURA == 7 and self.estado != 15:
        #                     self.timeVarredura = rospy.get_rostime().secs
        #                     self.VARREDURA = 8
        #                     print("esperando varredura")

        #                 elif self.VARREDURA == 8:
        #                     self.ROTINA_ORBS = 0
        #                     self.VARREDURA = 0
        #                     self.desce.publish(Empty())
        #                     print("entrar na rotina do orbs")

        #     elif status == self.cheguei:
        #         self.goToGlobal(flag=self.idle)
        #         self.timeStop = rospy.get_rostime().secs
        #         self.estado = 10

        #     elif status == self.busy:
        #         if self.estado != 2:
        #             self.estado = 2
        #             print("ocupado")

        # # ---------------------------- PERDEU ORBS SLAM ---------------------------------------
        # if self.PERDEU_ORBS >= 1 and self.startSM == 1:
        #     if self.PERDEU_ORBS == 1:
        #         self.goToGlobal(flag=self.abort)
        #         self.PERDEU_ORBS = 2

        #     elif status == self.abortei:
        #         self.timeStop = rospy.get_rostime().secs
        #         self.goToGlobal(flag=self.idle)
        #         self.PERDEU_ORBS = 3

        #     elif status == self.idle:
        #         if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
        #             if self.PERDEU_ORBS == 3:
        #                 self.timeRecOrb = rospy.get_rostime().secs
        #                 if rospy.get_rostime().secs - self.timeRecOrb > self.tempoRecOrb:
        #                     self.goToLocal(x=0.2)
        #                     self.PERDEU_ORBS = 4
        #             if self.PERDEU_ORBS == 4:
        #                 self.goToLocal(x=-0.2)
        #                 self.PERDEU_ORBS = 5
        #             if self.PERDEU_ORBS == 5:
        #                 self.PERDEU_ORBS = 3

        #     elif status == self.busy:
        #         if self.estado != 2:
        #             self.estado = 2
        #             print("ocupado")
            
        #     elif status == self.cheguei:
        #         self.estado = 6
        #         self.goToGlobal(flag=self.idle)
        #         self.timeStop = rospy.get_rostime().secs

        #     if orbs == self.comEscala:
        #         if self.PERDEU_ORBS < 4:
        #             self.PERDEU_ORBS = 0

        # # -------------------------- ORB SLAM -------------------------------------------------
        # if self.ROTINA_ORBS >= 1 and self.startSM == 1 and self.PERDEU_ORBS == 0:
        #     if status == self.busy:
        #         if self.estado != 2:
        #             self.estado = 2
        #             print("ocupado")

        #     elif status == self.cheguei:
        #         self.estado = 9
        #         self.goToGlobal(flag=self.idle)
        #         self.timeStop = rospy.get_rostime().secs

        #     elif status == self.idle:
        #         if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
        #             if self.estado == 9:
        #                 if self.ROTINA_ORBS == 1:
        #                     print("ligou orbs")
        #                     self.iniciaOrb.publish(0xFF)
        #                     self.ROTINA_ORBS = 4

        #                 elif self.ROTINA_ORBS == 4: # Vai pra direita
        #                     print("iniciou quadrado")
        #                     self.ROTINA_ORBS = 20 
        #                     self.estado = 7

        #                 elif self.ROTINA_ORBS == 30:  # vai pra frente
        #                     self.ROTINA_ORBS = 40
        #                     self.estado = 7

        #                 elif self.ROTINA_ORBS == 50: # vai pra esquerda
        #                     self.ROTINA_ORBS = 60
        #                     self.estado = 7

        #                 elif self.ROTINA_ORBS == 70: # vai pra tras
        #                     self.ROTINA_ORBS = 80
        #                     self.estado = 7

        #                 elif self.ROTINA_ORBS == 90: # vai pra tras
        #                     self.iniciaLog.publish(0XFF)
        #                     self.ROTINA_ORBS = 100
        #                     self.estado = 7
                    
        #             if self.estado == 7:
        #                 if self.ROTINA_ORBS == 20:
        #                     print("direita")
        #                     self.goToLocal(y=0.3 * self.direcaoOrbs * -1, t=3)
        #                     self.ROTINA_ORBS = 30
        #                     self.estado = 8

        #                 elif self.ROTINA_ORBS == 40:
        #                     print("frente")
        #                     self.goToLocal(x=0.3, t=3)
        #                     self.ROTINA_ORBS = 50
        #                     self.estado = 8

        #                 elif self.ROTINA_ORBS == 60:
        #                     print("esquerda")
        #                     self.goToLocal(y=0.3 * self.direcaoOrbs, t=3)
        #                     self.ROTINA_ORBS = 70
        #                     self.estado = 8

        #                 elif self.ROTINA_ORBS == 80:
        #                     print("tras")
        #                     self.goToLocal(x=-0.3, t=3)
        #                     self.ROTINA_ORBS = 90
        #                     self.estado = 8

        #                 elif self.ROTINA_ORBS == 100:
        #                     print("frente")
        #                     self.goToLocal(x=0.3, t=3)
        #                     self.ROTINA_ORBS = 13
        #                     self.estado = 8

        #             if orbs == self.comEscala:
        #                 self.estado = 6
        #                 self.goToLocal(x=self.rotas["x"][self.pos])
        #                 self.pos += 1
        #                 self.ROTACIONAR = 0
        #                 self.ROTINA_ORBS = 0

        # # ------------------------------------- ROTINA DIEGO --------------------------------------
        # if self.startSM == 1 and self.ROTINA_DIEGO >= 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS) == 0:
        #     if self.ROTINA_DIEGO == 60: # Se aproximar da base
        #         self.goToLocal(x=np.mean(self.targetPixel["x"]), y=np.mean(self.targetPixel["y"]), flag=self.desFinal)
        #         self.ROTINA_DIEGO = 61
        #     elif self.ROTINA_DIEGO == 50: # Pousar
        #         self.goToLocal(self.abort)
        #     elif self.ROTINA_DIEGO == 65:
        #         self.goToLocal(x=np.mean(self.targetPixel["x"]), y=np.mean(self.targetPixel["y"]), flag=self.calibraAlvo)

        #     if status == self.ackParam:
        #         self.ROTINA_DIEGO = 65

        #     elif status == self.abortei:
        #         self.estado = 6
        #         print("foi abortado")

        #         if self.ROTINA_DIEGO == 50:
        #             self.ROTINA_DIEGO = 7

        #         self.timeStop = rospy.get_rostime().secs
        #         self.goToGlobal(flag=self.idle)                    

        #     elif status == self.idle:
        #         if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
        #             self.estado = 6
        #             if self.ROTINA_DIEGO == 7:
        #                 self.goToGlobal(0,0,0,0,0,30) # land

        #                 if self.checarAlvo([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]]):
        #                     self.desce.publish(Empty())
        #                     self.dormir(t=10)
        
        #                     self.alvosBloqueados.append([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]])
                            
        #                     self.sobe.publish(Empty())
        #                     self.dormir(t=10)
                            
        #                     #self.goToLocal(self.curr_pos["x"], self.pontoAtual["y"], self.pontoAtual["z"])
        #                     self.ROTINA_DIEGO = 8
        #                 else:
        #                     self.ROTINA_DIEGO = 8
        #             if self.ROTINA_DIEGO == 8:
        #                 self.pos += 1 
        #                 self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
        #                 self.ROTINA_DIEGO == 0

        #     elif status == self.busy:
        #         if self.estado != 2:
        #             self.estado = 2
        #             print("ocupado")

        #     elif status == self.cheguei:
        #         self.estado = 6
        #         self.goToGlobal(flag=self.idle)
        #         self.timeStop = rospy.get_rostime().secs

        # # ------------------------------------- ROTINA NORMAL ------------------------------------------------------
        
        if self.startSM == 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS + self.ROTINA_DIEGO) == 0:
            if status == self.idle: # Idle
                if self.estado == 0:
                    if(self.takeOffBusy == 0):
                        self.sobe.publish(Empty())
                        self.takeOffBusy = 1
                        self.timeTakeOff = rospy.get_rostime().secs
                        print("subindo - takeoff")
                    else:
                        if(rospy.get_rostime().secs - self.timeTakeOff > self.tempoTakeOff):
                            self.estado = 1
                elif self.estado == 1:
                    self.goToLocal(0,0,2)
                    # self.goToGlobal(self.rotas["x"][0], self.rotas["y"][0], self.rotas["z"][0], self.rotas["yaw"][0])
                    self.estado = 8
                elif self.estado == 3:
                    if(rospy.get_rostime().secs - self.timeStop > self.tempoParar):
                        
                        self.stopBusy = 0  

                        '''
                        if self.pos == len(self.rotas["x"]):
                            if len(self.alvosBloqueados) != 5: # ja foi em todos os pontos e nao achou os alvos
                                self.pos = 1
                                rot = curr_pos["yaw"] * -1
                                self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
                                self.ROTACIONAR = 0
                                print('reiniciando a busca')
                            else: # ja foi em todos os pontos e achou os alvos
                                self.goToHome()
                                print('Voltando para a home')
                        else: 
                            if len(self.alvosBloqueados) != 5: # nao foi em todos os pontos e nao achou todos os alvos
                                # aki q entra os rotacionar ai em baixo
                                self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
                                self.pos += 1
                                self.ROTACIONAR = 0
                                print('foi rotacionado, vai para o proximo ponto')    
                            else: # nao foi em todos os pontos mas achou os alvos
                                self.goToHome()
                                print('Voltando para a home')
                        '''

                        if self.ROTACIONAR == 0: # Caso tenha chego em um waypoint mas ainda nao rotacionou
                            # Pouso Forcado
                            if self.rotas["x"][self.pos] == -90:
                                print("pousando")
                                self.desce.publish(Empty())
                                self.estado = 10
                            else:
                                if self.ROTACAO_ATUAL == 180:
                                    somaR = -90
                                elif self.ROTACAO_ATUAL == 0:
                                    somaR = 90
                                self.rotacionar(self.toRadian(somaR))

                                self.ROTACAO_ATUAL += somaR
                                self.estado = 9
                        elif self.ROTACIONAR == 1: # Caso tenha chegado em um waypoint e a rotacao esta correta
                            self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos], self.rotas["yaw"][self.pos])
                            self.pos += 1
                            self.ROTACIONAR = 0 
                            self.estado = 10
                    elif self.estado == 5:
                        if(rospy.get_rostime().secs - self.timeStop > self.tempoParar):
                            print("pousando")
                            self.desce.publish(Empty()) 
    
            elif status == self.cheguei: # Cheguei
                if self.stopBusy == 0:
                    print("cheguei")
                    self.goToGlobal(flag=self.idle)
                    print("Foi para o idle no cheguei")
                    self.stopBusy = 1
                    self.timeStop = rospy.get_rostime().secs
                    if self.estado == 2:
                        self.estado = 3

            elif status == self.busy: # Ocupado
                if self.estado == 9:
                    self.ROTACIONAR = 1
                if self.estado == 8:
                    self.ROTINA_ORBS = 1
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
