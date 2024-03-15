"""
Projeto 2EE - Manipulador Robótico

Nome: Everton da Silva Paiva

Curso: Engenharia de Controle e Automação

@GitHub: EvertonPaiva01

"""

#Importando as bibliotecas que serão utilizadas no projeto
from robot import Robot
def main():

    # Rotacionando as juntas do Manipulador Panda
    #Robot().flex_robot()

    # Definindo Posições para o manipulador
    #Robot().one_step_3d(0.2,0.3,0.35)

    # Cinemática
    Robot().kinematics()

    # Cinemática 3D
    Robot().kinematics_3d()

if __name__ == '__main__':
    main()