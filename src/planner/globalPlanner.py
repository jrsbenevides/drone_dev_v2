#!/usr/bin/env python

import math
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray
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
        self.rotas = {"x": [0, 3, 3, -90], "y": [0, 0.5, 2, -90], "z": [1.5, 1.5, 1.5, -90], "yaw": [0, 0, 0]}
        self.pos = 1
        self.ROTINA_DIEGO = 0
        self.LIMIAR_PRECISAO_ALVO = 0.8
        self.erroAlvo = 0.2
        self.focoCamera = {"x": 595.035494, "y": 594.674498} 
        self.target = {"x": [], "y": []}
        #self.alvo = {"local": {"x": 0.5, "y": 0.5, "z": 0, "yaw": 0}, "global": {"x": 2.5, "y": 0.75, "z": 2}}
        #self.guardaLocal = {"x": [], "y": [], "z": [], "yaw": []}
        self.ROTACIONAR = 1
        self.ajusteTempo = 4
        self.viconOne = 0
        self.pose_home = {"x": 1, "y": 0.75, "z": 0.5}
        self.psi, self.po, self.r = 0, 0, 0

        self.startSM = 0
        self.takeOffBusy = 0
        self.estado = 0
        self.stopBusy = 0
        self.timeStop = 0
        self.timeTakeOff = 0
        self.timeRecOrb = 0
        self.ROTINA_ORBS = 0
        self.PERDEU_ORBS = 0

        self.tempoParar = 3
        self.tempoTakeOff = 6
        self.alturaDrone = 3
        self.tempoRecOrb = 5

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

        self.cmd = rospy.Publisher("planejamento", PoseStamped, queue_size=10)
        self.iniciaOrb = rospy.Publisher("iniciaOrbSlam", UInt8, queue_size=10)
        self.statusPlanning_sub = rospy.Subscriber("statusPlanning", UInt8, self.callbackPosicao)
        self.perceptionTarget = rospy.Subscriber("/perception/target", PoseArray, self.callbackVisao)
        self.sobe = rospy.Publisher("/bebop/takeoff", Empty, queue_size=10)
        self.desce = rospy.Publisher("/bebop/land", Empty, queue_size=10)
        self.vicon_sub = rospy.Subscriber("/vicon/bebop/bebop", TransformStamped, self.callbackVicon)

        rospy.init_node('Global_Planner', anonymous=True)

        self.dormir()
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
        pose.pose.orientation.y = 0 if t == -1 else pose.pose.orientation.y
        pose.pose.orientation.z = flag

        if self.ROTINA_DIEGO != 5 and flag == self.vai:
            print("Indo para o ponto: " + str(pos_local))
            print("tempo: " + str(pose.pose.orientation.y))

        self.cmd.publish(pose)

    def goToGlobal(self, x=0, y=0, z=0, yaw=0, t=0, flag=10):
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

    def goToHome(self):
        # vai em que angulo para a home?
        self.goToGlobal(self.pose_home["x"], self.pose_home["y"], self.pose_home["z"])
        self.estado = 5

    def callbackVisao(self, target):
        if self.ROTINA_DIEGO == 0 or self.ROTINA_DIEGO >= 3:
            ix = []
            iy = []

            for i in range(len(target.pose)):
                ix.append(target.pose[i].position.x)
                iy.append(target.pose[i].position.y)

            flag = target.pose.orientation.w

            if ix > 20 or self.ROTINA_DIEGO >= 3: #adjust
                for i in range(len(ix)):
                    self.target["x"].append(ix[i] * self.curr_pos["z"] / self.focoCamera["x"])
                    self.target["y"].append(iy[i] * self.curr_pos["z"] / self.focoCamera["y"])
                self.goToGlobal(flag=self.abort)
                if self.ROTINA_DIEGO == 0:
                    self.ROTINA_DIEGO = 1
            
    def callbackPosicao(self, data):
        orbs = (0b11000000 & data.data) >> 6
        status = (0b00001111 & data.data)

        if orbs == self.perdeu and (self.ROTINA_DIEGO != 5 and self.ROTINA_DIEGO != 3):
            self.PERDEU_ORBS = 1
        
        # ---------------------------- PERDEU ORBS SLAM ---------------------------------------
        if self.PERDEU_ORBS >= 1 and self.startSM == 1:
            if self.PERDEU_ORBS == 1:
                self.goToGlobal(flag=self.abort)
                self.PERDEU_ORBS = 2

            elif status == self.abortei:
                self.timeStop = rospy.get_rostime().secs
                self.goToGlobal(flag=self.idle)
                self.PERDEU_ORBS = 3

            elif status == self.idle:
                if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
                    if self.PERDEU_ORBS == 3:
                        self.timeRecOrb = rospy.get_rostime().secs
                        if rospy.get_rostime().secs - self.timeRecOrb > self.tempoRecOrb:
                            self.goToLocal(x=0.2)
                            self.PERDEU_ORBS = 4
                    if self.PERDEU_ORBS == 4:
                        self.goToLocal(x=-0.2)
                        self.PERDEU_ORBS = 5
                    if self.PERDEU_ORBS == 5:
                        self.PERDEU_ORBS = 3

            elif status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")
            
            elif status == self.cheguei:
                self.estado = 6
                self.goToGlobal(flag=self.idle)
                self.timeStop = rospy.get_rostime().secs

            if orbs == self.comEscala:
                if self.PERDEU_ORBS < 4:
                    self.PERDEU_ORBS = 0

        # -------------------------- ORB SLAM -------------------------------------------------
        if self.ROTINA_ORBS >= 1 and self.startSM == 1 and self.PERDEU_ORBS == 0:
            if status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")

            elif status == self.cheguei:
                self.estado = 6
                self.goToGlobal(flag=self.idle)
                self.timeStop = rospy.get_rostime().secs

            elif status == self.idle:
                self.estado = 6
                if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
                    if self.ROTINA_ORBS == 1:
                        self.iniciaOrb.publish(0xFF)
                        self.ROTINA_ORBS = 4
                    if self.ROTINA_ORBS == 4:
                        self.iniciaOrb.publish(0x00)
                        self.ROTINA_ORBS = 0
                        self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
                        self.pos += 1
                        self.ROTACIONAR = 0
                        #self.timeRecOrb = rospy.get_rostime().secs
                        #if rospy.get_rostime().secs - self.timeRecOrb > self.tempoRecOrb:
                        #    self.ROTINA_ORBS = 2

                    if self.ROTINA_ORBS == 5:
                        self.ROTINA_ORBS = 3
                    if self.ROTINA_ORBS == 6:
                        self.ROTINA_ORBS = 2
                       
            if orbs == self.comEscala and (self.ROTINA_ORBS == 2 or self.ROTINA_ORBS == 3):
                self.estado = 6
                self.goToGlobal(x=self.rotas["x"][self.pos], y=self.rotas["y"][self.pos], z=self.rotas["z"][self.pos])
                self.pos += 1
                self.ROTACIONAR = 0
                self.ROTINA_ORBS = 0

            elif orbs == self.semEscala and (self.ROTINA_ORBS == 2 or self.ROTINA_ORBS == 3):
                self.estado = 6
                if self.ROTINA_ORBS == 2:
                    self.goToLocal(x=0.3)
                    self.ROTINA_ORBS = 5
                elif self.ROTINA_ORBS == 3:
                    self.goToLocal(x=-0.3)
                    self.ROTINA_ORBS = 6

            elif orbs == self.iniciouOrb:
                if self.ROTINA_ORBS == 2:
                    self.timeRecOrb = rospy.get_rostime().secs
                    if rospy.get_rostime().secs - self.timeRecOrb > self.tempoRecOrb:
                        self.ROTINA_ORBS = 2


        # ------------------------------------- ROTINA DIEGO --------------------------------------
        if self.startSM == 1 and self.ROTINA_DIEGO >= 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS) == 0:
            if status == self.abortei:
                self.estado = 6
                print("foi abortado")

                if self.ROTINA_DIEGO == 6:
                    self.ROTINA_DIEGO = 7

                elif self.ROTINA_DIEGO == 1:
                    self.pontoAtual["x"] = self.curr_pos["x"]
                    self.pontoAtual["y"] = self.curr_pos["y"]
                    self.pontoAtual["z"] = self.curr_pos["z"]

                    self.ROTINA_DIEGO = 2

                self.timeStop = rospy.get_rostime().secs
                self.goToGlobal(flag=self.idle)                    

            elif status == self.idle:
                if (rospy.get_rostime().secs - self.timeStop > self.tempoParar):
                    self.estado = 6
                    if self.ROTINA_DIEGO == 2:
                        self.goToLocal(x=min(self.target["x"]), y=min(self.target["y"]))
                        self.ROTINA_DIEGO == 3
                    if self.ROTINA_DIEGO == 3:
                        self.goToLocal(x=min(self.target["x"]), y=min(self.target["y"]), t=-1, flag=self.desFinal)
                        self.ROTINA_DIEGO = 4
                    if self.ROTINA_DIEGO == 7:
                        self.goToGlobal(0,0,0,0,0,30) # land

                        if self.checarAlvo([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]]):
                            self.desce.publish(Empty())
                            self.dormir(t=10)
        
                            self.alvosBloqueados.append([self.curr_pos["x"], self.curr_pos["y"], self.curr_pos["z"]])
                            
                            self.sobe.publish(Empty())
                            self.dormir(t=10)
                            
                            self.goToLocal(self.curr_pos["x"], self.pontoAtual["y"], self.pontoAtual["z"])
                            self.ROTINA_DIEGO = 8
                        else:
                            self.ROTINA_DIEGO = 8
                    if self.ROTINA_DIEGO == 8:
                        self.goToGlobal(self.rotas["x"][self.pos], self.rotas["y"][self.pos], self.rotas["z"][self.pos])
                        self.ROTINA_DIEGO == 0

            elif status == self.busy:
                if self.estado != 2:
                    self.estado = 2
                    print("ocupado")

            elif status == self.cheguei:
                self.estado = 6
                self.goToGlobal(flag=self.idle)
                self.timeStop = rospy.get_rostime().secs

            elif status == self.ackParam:
                self.ROTINA_DIEGO = 5
                self.estado = 6

            if self.ROTINA_DIEGO == 5:
                self.goToLocal(x=min(self.target["x"]), y=min(self.target["y"]), t=-1, flag=self.calibraAlvo)
                if min(self.target["x"]) + min(self.target["y"]) < self.erroAlvo:
                    self.goToLocal(flag=self.abort)
                    self.ROTINA_DIEGO = 6

        # ------------------------------------- ROTINA NORMAL ------------------------------------------------------
        
        if self.startSM == 1 and (self.ROTINA_ORBS + self.PERDEU_ORBS + self.ROTINA_DIEGO) == 0:
            self.iniciaOrb.publish(0x00)
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
                    self.goToGlobal(self.rotas["x"][0], self.rotas["y"][0], self.rotas["z"][0], self.rotas["yaw"][0])
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
