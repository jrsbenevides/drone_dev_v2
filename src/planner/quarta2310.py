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
        self.alvosBloqueados = [[0.75, 1, 0.5], [3.5, 0.75, 2], [7.5, 7.5, 2]]
        self.alturaDrone = 1.5
        self.localPerdeuOrbs = {"x": 0, "y": 0, "z": 0, "yaw": 0}
        # So para frente
        #self.rotas = {"x": [0, 4.5, -90], "y": [0, 1.5, -90], "z": [self.alturaDrone, self.alturaDrone, -90], "yaw": [0, 0, -90]}
        #self.rotas = {"x": [0, 4, 0.75, -90], "y": [0, 1.5, 1.5, -90], "z": [self.alturaDrone, self.alturaDrone, self.alturaDrone, -90], "yaw": [0, 0, 0, -90]}
        
        # Primeiro na suspensa
        self.rotas = {"x": [0, -80, -90], "y": [0, 0, -90], "z": [self.alturaDrone, self.alturaDrone, -90], "yaw": [0, math.pi/2, -90]}
        
        # Triangular
        #self.rotas = {"x": [0, 4, 6, 0.75, -90], "y": [0, 1, 6, 1, -90], "z": [self.alturaDrone, self.alturaDrone, self.alturaDrone, self.alturaDrone, -90], "yaw": [0, 0, 0, 0, -90]}
        self.pos = 0
        self.ROTINA_DIEGO = 0
        self.LIMIAR_PRECISAO_ALVO = 1
        self.focoCamera = {"x": 595.035494, "y": 594.674498} 
        self.ROTACIONAR = 1
        self.ajusteTempo = 5
        self.viconOne = 0
        self.pose_home = {"x": 1, "y": 0.75, "z": 0.5}
        self.psi, self.po, self.r = 0, 0, 0
        self.direcaoOrbs = 1 # 1 = esquerda e -1 = direita
        self.anguloCameraGrau = 40
        self.estrategia = 1
        self.tamBaseBaixo = 50 # numero de pixels pegados para o PID
        self.pixelsPouso = 0.2
        self.psi0 = 0
        self.po = {"x": 0, "y": 0, "z": 0}
        self.zerarOdom = {"x": 0, "y": 0, "z": 0}

        self.startSM = 0
        self.takeOffBusy = 0
        self.estado = 0
        self.stopBusy = 0
        self.timeStop = 0
        self.timeTakeOff = 0
        self.timeLand = 0
        self.timeRecOrb = 0
        self.timePorbs = 0
        self.timeTubo = 0
        self.timePid = 0
        self.timeAcabouOrbs = 0
        self.timeVarredura = 0
        self.timeBaseBaixo = 0
        self.ROTINA_ORBS = 0
        self.PERDEU_ORBS = 0
        self.VARREDURA = 0
        self.usingOrbs = 0
        self.ignoraDiego = 0
        self.uniqueBaseBaixo = 0
        self.vezPid = 0
        self.acabouOrbs = 0
        self.pousoSuspenso = 0 # 1 se quiser fazer o pouso suspenso primeiro

        self.tempoParar = 2.5 # n deixa isso menor que 2
        self.tempoTakeOff = 6
        self.tempoLand = 6
        self.tempoRecOrb = 5
        self.tempoVarredura = 5
        self.tempoPOrbs = 6
        self.tempoBaseBaixo = 15
        self.tempoTubo = 7
        self.tempoPid = 0.2
        self.tempoAcabouOrbs = 19 # 19 se nao for na suspensa e 4 se for na suspensa primeiro

        self.idle = 0
        self.vai = 10
        self.land = 30
        self.perdeuOrbs = 44
        self.perdeuOrbsSempre = 47
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

        self.posicaoAtual = {"log": {"x": 0, "y": 0, "z": 0, "yaw": 0}, "odom": {"x": 0, "y": 0, "z": 0, "yaw": 0}}

        self.tt = 0
        self.tempolimite = 60*10

        self.cmd = rospy.Publisher("planejamento", PoseStamped, queue_size=10)
        self.iniciaOrb = rospy.Publisher("iniciaOrbSlam", UInt8, queue_size=10)
        self.iniciaLog = rospy.Publisher("iniciaLog", UInt8, queue_size=10)
        self.myCamera   = rospy.Publisher("/bebop/camera_control", Twist, queue_size = 1)
        self.sobe = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.desce = rospy.Publisher("/bebop/land", Empty, queue_size=10)

        self.statusOdometry_sub = rospy.Subscriber("/bebop/odom", Odometry, self.callbackOdom)
        self.statusLog_sub = rospy.Subscriber("/scale/log", Odometry, self.callbackLog)
        self.statusPlanning_sub = rospy.Subscriber("statusPlanning", UInt8, self.callbackPosicao)
        #self.statusVisao_sub = rospy.Subscriber("/perception/target", PoseArray, self.callbackVisao)
        
        # self.vicon_sub = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, self.callbackVicon)
    
        rospy.init_node('Global_Planner', anonymous=True)
        self.dormir(t=1)

        twist = Twist()
        twist.angular.y = -80
        self.myCamera.publish(twist)

        self.tt = self.time()
        self.dormir(t=1)
        self.startSM = 1
        print("comecou")

    def dormir(self, t=5):
        rospy.sleep(t)

    def rotacionar(self, angle, t=4):
        pose = PoseStamped()

        # print("entrei na rotacao")
        
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 0

        pose.pose.orientation.x = angle
        if abs((angle) - (math.pi/2)) < 0.1:
            pose.pose.orientation.y = t
            # print("menor")
        else:
             pose.pose.orientation.y = t * self.toGrau(angle) / 90
             # print("maior")

        pose.pose.orientation.z = self.vai

        # print("mandou tudo")
        
        self.ROTACAO_ATUAL += self.toGrau(angle)
        print("Girando por " + str(pose.pose.orientation.y) + "segundos")
        self.estado == 15
        
        self.cmd.publish(pose)

    def zerarOdometria(self, x=-10, y=-10, z=-10):
        if x != 10:
            self.zerarOdom["x"] = x - self.curr_pos["x"]

        if y != 10:
            self.zerarOdom["y"] = y - self.curr_pos["y"]

        if z != 10:
            self.zerarOdom["z"] = z - self.curr_pos["z"]

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

    def goToLocal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
        flag = self.vai if flag == 10 else flag
        # angle = self.curr_pos["yaw"]#self.toRadian(self.ROTACAO_ATUAL) 

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # if yaw == 0:
        #     pose.pose.orientation.x = angle - self.curr_pos["yaw"]
        # else:
        pose.pose.orientation.x = yaw
        if t == 0:
            tempo = math.ceil(self.deftempo(x, y, z))
            if(tempo<5):
                t = 5
            else:
                t = tempo
        pose.pose.orientation.y = t
        pose.pose.orientation.z = flag

        if flag == self.vai:
            print("x: " + str(pose.pose.position.x) + " y: " + str(pose.pose.position.y) + " z: " + str(pose.pose.position.z))
            print("tempo: " + str(pose.pose.orientation.y))

        self.cmd.publish(pose)

    def goToGlobal(self, x=-100, y=-100, z=-100, yaw=0, t=0, flag=10):
        x1 = x - self.curr_pos["x"] if x != -100 else 0
        y1 = y - self.curr_pos["y"] if y != -100 else 0
        z1 = z - self.curr_pos["z"] if z != -100 else 0
        psi0 = self.curr_pos["yaw"]
        yawLocal = yaw - psi0 
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
        pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
        
        self.goToLocal(pos_local[0], pos_local[1], pos_local[2], yawLocal, t, flag)

    def checarAlvo(self, destino):
        bloqueado = 0
        a2 = self.deftempo(destino[0], destino[1], 0) / self.ajusteTempo
        
        for i in range(len(self.alvosBloqueados)):
            a1 = self.deftempo(self.alvosBloqueados[i][0], self.alvosBloqueados[i][1], 0) / self.ajusteTempo
            
            if abs(a1 - a2) < self.LIMIAR_PRECISAO_ALVO:
                bloqueado += 1

        if bloqueado == 0:
            return True
            print("alvo ainda n foi visitado, pode olhar")
        else:
            return False 
            print("Esse alvo ja foi visitado")

    def callbackLog(self, log):
        self.posicaoAtual["log"]["x"] = log.pose.pose.position.x #+ 0.75
        self.posicaoAtual["log"]["y"] = log.pose.pose.position.y #+ 1
        self.posicaoAtual["log"]["z"] = log.pose.pose.position.z #+ 0.5
        roll, pitch, yaw = euler_from_quaternion([log.pose.pose.orientation.x, log.pose.pose.orientation.y, log.pose.pose.orientation.z, log.pose.pose.orientation.w])
        self.posicaoAtual["log"]["yaw"] = yaw

        if self.usingOrbs == 1:
            self.curr_pos["x"] = log.pose.pose.position.x #+ 0.75
            self.curr_pos["y"] = log.pose.pose.position.y #+ 1
            self.curr_pos["z"] = log.pose.pose.position.z #+ 0.5
            self.curr_pos["yaw"] = yaw

    def callbackOdom(self, odom):
        roll, pitch, yaw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])        
        
        if self.viconOne == 0:
            self.psi0 = yaw
            self.po["x"] = odom.pose.pose.position.x
            self.po["y"] = odom.pose.pose.position.y
            self.po["z"] = odom.pose.pose.position.z
            self.viconOne = 1
        else:
            if self.usingOrbs == 0:
                self.r = [[np.cos(self.psi0), np.sin(self.psi0) * -1, 0], [np.sin(self.psi0), np.cos(self.psi0), 0], [0, 0, 1]]
                posicao = np.dot(np.transpose(np.asarray(self.r)), np.asarray([[odom.pose.pose.position.x - self.po["x"]], [odom.pose.pose.position.y - self.po["y"]], [odom.pose.pose.position.z - self.po["z"]]]))
                self.curr_pos["x"] = posicao[0] #+ 0.75 + self.zerarOdom["x"]
                self.curr_pos["y"] = posicao[1] #+ 1 + self.zerarOdom["y"]
                self.curr_pos["z"] = posicao[2] #+ 0.5 + self.zerarOdom["z"]
                self.curr_pos["yaw"] = yaw - self.psi0

        self.posicaoAtual["odom"]["x"] = odom.pose.pose.position.x #+ 0.75 + self.zerarOdom["x"]
        self.posicaoAtual["odom"]["y"] = odom.pose.pose.position.y #+ 1 + self.zerarOdom["y"]
        self.posicaoAtual["odom"]["z"] = odom.pose.pose.position.z #+ 0.5 + self.zerarOdom["z"]
        self.posicaoAtual["odom"]["yaw"] = yaw

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

        ran = self.curr_pos["x"] < 2 and self.curr_pos["y"] < 2.5 # Se for verdade ta na base costeira
        if self.startSM == 1 and ran == False: 
            if self.checarAlvo([self.curr_pos["x"], self.curr_pos["y"]]) == True:
                u1, v1, area1, yaw1, m1, flag1 = [], [], [], [], [], []
                for pose in target.poses:
                    u1.append(pose.position.x)
                    v1.append(pose.position.y)
                    area1.append(pose.position.z)
                    yaw1.append(pose.orientation.y)
                    m1.append(pose.orientation.z)
                    flag1.append(pose.orientation.w)

                a = np.where(np.asarray(flag1) == self.baseToda)
                b = np.where(np.asarray(flag1) == self.parteBaixo)

                if len(a[0]) != 0:
                    self.ROTINA_DIEGO = 50 # flag para pousar na rotina do diego
                    print("viu a base toda, partiu pousar")
                elif len(b[0]) != 0:
                    # Rotina sem PID
                    if self.uniqueBaseBaixo == 0:
                        self.ROTINA_DIEGO = 60 # flag para se aproximar na rotina do diego
                        self.u = []
                        self.v = []
                        self.area = []
                        self.yaw = []
                        self.m = []
                        self.flag = []
                        self.timeBaseBaixo = 0
                        self.uniqueBaseBaixo = 1
                        self.goToLocal(flag=self.abort)
                        print("rotina de aproximcao inicial")

                    if self.timeBaseBaixo < self.tamBaseBaixo:
                        self.timeBaseBaixo += 1
                        
                        for i in b[0]:
                            self.u.append(u1[i])
                            self.v.append(v1[i])
                            self.area.append(area1[i])
                            self.yaw.append(yaw1[i])
                            self.m.append(m1[i])
                            self.flag.append(flag1[i])
                
    def callbackPosicao(self, data):
        status = (0b00001111 & data.data)

        # ------------------------------------- ROTINA DIEGO --------------------------------------
        if self.startSM == 1 and self.ROTINA_DIEGO >= 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS) == 0:
            if status == self.abortei:
                self.goToGlobal(flag=self.idle)
                self.ROTINA_DIEGO = 10
                print("foi abortado na rotina do diego")

            elif status == self.busy:
                if self.estado not in [699, 700, 701]:
                    if self.estado != 2 :
                        self.estado = 2
                        print("ocupado")

            elif status == self.cheguei:
                print("cheguei")
                if self.ROTINA_DIEGO == 100:
                    self.ROTINA_DIEGO = 50 # ir pousar
                    print("indo pousar")
                self.goToLocal(flag=self.idle)
                self.timeStop = self.time()

            elif status == self.idle:
                if self.ROTINA_DIEGO == 50:
                    if self.estado == 699: # se ainda tiver pousado nessa base
                        print("alvo ainda n foi visitado")
                        self.desce.publish(Empty())
                        print("pousou")
                        self.timeLand = self.time()
                        self.estado = 700
                    elif self.estado == 700 and self.time() - self.timeLand > self.tempoLand:
                        self.alvosBloqueados.append([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]])
                        print("subiu")
                        self.sobe.publish(Empty())
                        self.timeTakeOff = self.time()
                        self.estado = 701
                    elif self.estado == 701 and self.time() - self.timeTakeOff > self.tempoTakeOff:
                        print("ajeitando altitude")
                        self.goToGlobal(z=self.alturaDrone)
                        self.ROTACIONAR = 0
                        self.pos -= 1
                        self.estado = 6
                        self.uniqueBaseBaixo = 0 # libera para comecar a enxergar outro alvo
                    
                if self.ROTINA_DIEGO == 10:
                    #print("entrando na rotina de aproximacao em pixel")
                    if self.timeBaseBaixo == self.tamBaseBaixo:
                        self.timeBaseBaixo += 1
                        self.targetPixel["x"] = (np.mean(self.u) * (self.curr_pos["z"]) / self.focoCamera["x"]) + 0.3
                        self.targetPixel["y"] = np.mean(self.v) * (self.curr_pos["z"]) / self.focoCamera["y"]
                        print("viu o alvo")
                        if self.checarAlvo([self.targetPixel["x"], self.targetPixel["y"]]):
                            print("se aproximando do alvo")
                            self.goToLocal(self.targetPixel["x"], self.targetPixel["y"], t=6)
                        else:
                            print("alvo ja foi visitado")
                            self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos])
                        self.estado = 6 # numero para evitar entrar em algo que nao deveria
                        
                        self.ROTINA_DIEGO = 100

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
                    self.goToGlobal(z=self.rotas["z"][self.pos])
                    # Comentar se for comecar com a base suspensa
                    if self.pousoSuspenso == 0:
                        self.acabouOrbs = 1
                        self.timeAcabouOrbs = self.time()

                    self.pos += 1
                    self.estado = 20 # 8 se quiser entrar no orbs # qualquer flag que nao seja 5 e 9 segue a vida
                elif self.estado == 3 or self.estado == 13:
                    if(self.time() - self.timeStop > self.tempoParar): 
                        print("ROTACIONAR:" + str(self.ROTACIONAR))
                        if self.ROTACIONAR == 0: # Caso tenha chego em um waypoint mas ainda nao rotacionou
                            # Pouso Forcado
                            if self.rotas["x"][self.pos] == -90:
                                print("pousando")
                                self.desce.publish(Empty())
                                self.startSM = 0
                            elif self.rotas["yaw"][self.pos] == -70:
                                print("pousando na base suspensa")
                                self.desce.publish(Empty())
                                self.timeLand = self.time()
                                self.ROTACIONAR = 1
                                #self.pos += 1
                            else:
                                self.ROTACIONAR = 2 # Flag para seguir trajetoria
                        elif self.ROTACIONAR == 1: # Caso tenha chegado em um waypoint e a rotacao esta correta
                            # Espaco caso queira fazer algo entre os alvos
                            if self.time() - self.timeLand > self.tempoLand:
                                if self.rotas["yaw"][self.pos] == -70:
                                    print("takeoff")
                                    self.sobe.publish(Empty())
                                    #self.pos += 1
                                    self.timeTakeOff = self.time()
                                    self.ROTACIONAR = 3
                                else:
                                    self.ROTACIONAR = 2
                            #self.estado = 10
                        elif self.ROTACIONAR == 2:
                            if self.rotas["x"][self.pos] == -80:
                                self.rotacionar(angle=self.rotas["yaw"][self.pos])
                                print("ESTOU ROTACIONANDO!!! ULTIMO ESTAGIO")
                            else:
                                self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos])
                                print("ESTOU INDO PRA UM PONTO GLOBAL")
                            self.pos += 1
                            self.ROTACIONAR = 0
                            self.estado = 20
                        elif self.ROTACIONAR == 3:
                            if self.time() - self.timeTakeOff > self.tempoTakeOff:
                                print("volta pra rota")
                                self.ROTACIONAR = 2
                                if self.pousoSuspenso == 1:
                                    self.acabouOrbs = 1
                                    self.timeAcabouOrbs = self.time()
                                self.pos += 1

                
                if self.estado == 5: # Flag para pousar
                    print("pousando")
                    self.desce.publish(Empty()) 
    
            elif status == self.cheguei: # Cheguei
                #if self.stopBusy == 0:
                print("cheguei")
                self.goToGlobal(flag=self.idle)
                self.ROTACIONAR = 0
                self.timeStop = self.time()
                if self.estado == 2: # vai para flag 3 para seguir trajetoria
                    self.estado = 3

            elif status == self.busy: # Ocupado
                # eu apaguei um estado == 9 equivale a rotacionar = 1 (n lembro pra que servia)
                if self.estado == 8: # quando inicia pela primeira vez, qual rotina vai seguir
                    self.ROTINA_ORBS = 1
                if self.estado != 2:
                    print("estou ocupado")
                    if self.estado != 5: #vai pousar, entao nao pode troacr flag
                        self.estado = 2
            
def main():
    globalPlanner()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()



