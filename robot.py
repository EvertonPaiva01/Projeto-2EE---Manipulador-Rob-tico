import numpy as np
import roboticstoolbox as rtb
import swift
import numpy as np
import spatialmath as sm
import spatialgeometry as sg
from spatialmath import SE3

class Robot():
    def __init__(self):
        # Definição dos Parâmetros D-H como Variáveis
        self.a = [0, 0, 0.088, -0.088, 0, 0, 0.088]  # Comprimento dos links
        self.alpha = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2]  # Ângulo ao redor de x
        self.d = [0.333, 0, 0, 0.316, 0, 0.384, 0.107]  # Deslocamento ao longo de z
        self.theta = [0, -np.pi/2, 0, 0, 0, 0, 0]  # Ângulo ao redor de z


    # Rotacionar todos os eixos eixos do robô
    def flex_robot(self):
        # # Criando o robô com os parâmetros definidos
        panda = rtb.DHRobot(
            [rtb.RevoluteDH(d=self.d[i], a=self.a[i], alpha=self.alpha[i], offset=self.theta[i]) for i in range(7)],
            name='Panda'
        )
        return panda.teach(None,True)
    
    # Dado o ponto (x,y,z) em centimetro, o manipulador vai até o ponto e retorna para origem
    def one_step_3d(self,x,y,z):
        #pos_x -> linha vermelha
        #pos_y -> linha verde
        #pos_z -> linha azul

        # Inicializa o ambiente Swift
        env = swift.Swift()
        env.launch(realtime=True)
        panda = rtb.models.Panda()
        print(panda)
        panda.q = panda.qr  # Configuração inicial
        env.add(panda)
        
        # Define o ponto designado a partir da posição atual
        Tep = panda.fkine(panda.q) * sm.SE3.Tx(x) * sm.SE3.Ty(y) * sm.SE3.Tz(z)
        print(Tep)
        axes = sg.Axes(length=0.1, base=Tep)
        env.add(axes)

        arrived = False

        # Configurações para o loop
        dt = 0.01

        while not arrived:
            v, arrived = rtb.p_servo(panda.fkine(panda.q), Tep,gain=1,threshold=dt)
            j = panda.jacobe(panda.q)
            panda.qd = np.linalg.pinv(j) @ v
            env.step(dt)
        # Calcula a solução de cinemática inversa para o ponto designado (para validação)
        sol = panda.ikine_LM(Tep)
        print("\nPosição final alcançada:\n", sol.q)

        # Move o robô de volta para a posição inicial
        print("\nMovendo de volta para a posição inicial...")
        T_initial = panda.fkine(panda.qr)  # Posição inicial
        arrived = False
        while not arrived:
            v, arrived = rtb.p_servo(panda.fkine(panda.q), T_initial,gain=1,threshold=dt)
            j = panda.jacobe(panda.q)
            panda.qd = np.linalg.pinv(j) @ v
            env.step(dt)

        # Fecha a janela após a visualização
        env.close()

        # Mantém a janela aberta para visualização
        #env.hold()


    def kinematics_3d(self):
        # Exibindo os parametros
        robot = rtb.models.DH.Panda()
        print('Parâmetros D-H:\n')
        print(robot)
        T = robot.fkine(robot.qz)
        print('\nMatriz de Transformação:\n')
        print(T)

        # Matriz de transformação inversa
        T_inv = T.inv()
        print("\nMatriz de Transformação Inversa:")
        print(T_inv)


        # Inicializa o ambiente Swift para visualização 3D
        env = swift.Swift()
        env.launch(realtime=True)

        # Carrega o modelo do robô Panda e adiciona ao ambiente
        panda = rtb.models.Panda()  # Utiliza o modelo Denavit-Hartenberg do Panda
        panda.q = panda.qr  # Configuração inicial
        env.add(panda) 
        # Função para mover o robô até um determinado ponto usando cinemática inversa
        def move_robot_ik(target):
            sol = panda.ikine_LM(target)  # Resolve a cinemática inversa para o ponto alvo
            traj = rtb.jtraj(panda.q, sol.q, 50)  # Planeja uma trajetória de junta

            for q in traj.q:
                panda.q = q  # Atualiza a configuração do robô
                env.step()  # Atualiza a visualização

        # Função para simular movimento direto baseado em trajetórias predefinidas
        def move_robot_direct(traj):
            for q in traj.q:
                panda.q = q
                env.step()

        # Exemplo de cinemática direta: mover o robô para uma nova configuração
        q_new = np.array([-0.95370207,1.2067317,1.01104287,-1.35567777,-1.07129655,2.09011276, 0.9774728 ])
        print('\nCinemática Direta\n')

        # Definindo o Trajeto
        traj_direct = rtb.jtraj(panda.q, q_new, 50)
        # Movendo
        move_robot_direct(traj_direct)
        # Voltando para horigem
        move_robot_ik(panda.fkine(panda.qr))


        # Exemplo de cinemática inversa: mover o robô para uma posição e orientação específicas
        print('\nCinemática Inversa\n')
        Tep = panda.fkine(panda.q) * sm.SE3.Tx(0.2) * sm.SE3.Ty(0.2) * sm.SE3.Tz(0.35)
        move_robot_ik(Tep)

        # Exemplo de planejamento de trajetória: volta para a posição inicial
        move_robot_ik(panda.fkine(panda.qr))  # Usando cinemática inversa para voltar à posição inicial

        # Mantém a janela aberta para visualização
        env.hold()

    def kinematics(self):
        # # Criando o robô com os parâmetros definidos
        panda = rtb.DHRobot(
            [rtb.RevoluteDH(d=self.d[i], a=self.a[i], alpha=self.alpha[i], offset=self.theta[i]) for i in range(7)],
            name='Panda'
        )
        # Configuração inicial de junta
        qz = np.zeros(7)

        # Visualizar o robô e frames de coordenadas
        print("Visualizando o robô na configuração inicial...")
        panda.plot(qz, block=True)

        # Imprimir a tabela de Denavit-Hartenberg
        print("\nTabela de Denavit-Hartenberg do Panda:")
        print(panda)

        # Calcular e imprimir a matriz de transformação para a configuração inicial
        T = panda.fkine(qz)
        print("\nMatriz de Transformação Final para a configuração inicial:")
        print(T)

        # Definir uma configuração final
        q_final = np.array([-0.95370207,1.2067317,1.01104287,-1.35567777,-1.07129655,2.09011276, 0.9774728 ])

        # Planejar e visualizar uma trajetória
        qt = rtb.jtraj(qz, q_final, 50)
        print("\nVisualizando a trajetória de planejamento...")
        panda.plot(qt.q, block=True)

        # Usar a configuração final para calcular a cinemática direta
        T_direct = panda.fkine(q_final)
        print("\nMatriz de Transformação da Cinemática Direta para a configuração final:")
        print(T_direct)
        
        # Utilizar a matriz de transformação direta como objetivo para a cinemática inversa
        sol_inv = panda.ikine_LM(T_direct)
        q_inv = sol_inv.q

        # Verificar a configuração resultante
        T_inv_check = panda.fkine(q_inv)
        print("\nVerificação da Pose alcançada pela Cinemática Inversa:")
        print(T_inv_check)

        # Visualizar a configuração resultante
        print("\nVisualizando a configuração alcançada pela Cinemática Inversa...")
        panda.plot(q_inv, block=True)