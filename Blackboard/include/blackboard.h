/*--------------------------------------------------------------------

******************************************************************************
* @file blackboard.h
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
* @version V0.0.0
* @created 07/04/2014
* @Modified 26/05/2015
* @e-mail isaac25silva@yahoo.com.br
* @brief Main header black board
****************************************************************************

Arquivo de cabeçalho contendo as funções e definições do black board

/--------------------------------------------------------------------*/

#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#define KEY 10345678 //Chave de acesso a memoria

//---- Definições da memória compartilhada------------------------------
#define PLANNING_COMMAND *(mem)
#define PLANNING_PARAMETER_VEL *(mem+1)
#define PLANNING_PARAMETER_ANGLE *(mem+2)
#define IMU_STATE *(mem+3)


#define CONTROL_ACTION *(mem+13)
#define CONTROL_HEIGHT_A *(mem+14)
#define CONTROL_HEIGHT_B *(mem+15)
#define CONTROL_HEIGHT_C *(mem+16)
#define DECISION_ACTION_A *(mem+17)
#define DECISION_ACTION_B *(mem+18)
#define DECISION_STATE *(mem+19)
#define DECISION_POSITION_A *(mem+20)
#define DECISION_POSITION_B *(mem+21)
#define DECISION_POSITION_C *(mem+22)
#define DECISION_BALL_POS *(mem+23)
#define DECISION_OPP1_POS *(mem+24)
#define DECISION_OPP2_POS *(mem+25)
#define DECISION_OPP3_POS *(mem+26)
#define COM_ACTION_ROBOT1 *(mem+27)
#define COM_ACTION_ROBOT2 *(mem+28)
#define COM_ACTION_ROBOT3 *(mem+29)
#define COM_STATE_ROBOT1 *(mem+30)
#define COM_STATE_ROBOT2 *(mem+31)
#define COM_STATE_ROBOT3 *(mem+32)
#define COM_POS_ROBOT1 *(mem+33)
#define COM_POS_ROBOT2 *(mem+34)
#define COM_POS_ROBOT3 *(mem+35)
#define COM_POS_BALL_ROBOT1 *(mem+36)
#define COM_POS_BALL_ROBOT2 *(mem+37)
#define COM_POS_BALL_ROBOT3 *(mem+38)
#define COM_POS_OPP_A_ROBOT1 *(mem+39)
#define COM_POS_OPP_A_ROBOT2 *(mem+40)
#define COM_POS_OPP_A_ROBOT3 *(mem+41)
#define COM_POS_OPP_A_ROBOT4 *(mem+42)
#define COM_POS_OPP_B_ROBOT1 *(mem+43)
#define COM_POS_OPP_B_ROBOT2 *(mem+44)
#define COM_POS_OPP_B_ROBOT3 *(mem+45)
#define COM_POS_OPP_B_ROBOT4 *(mem+46)
#define COM_POS_OPP_C_ROBOT1 *(mem+47)
#define COM_POS_OPP_C_ROBOT2 *(mem+48)
#define COM_POS_OPP_C_ROBOT3 *(mem+49)
#define COM_POS_OPP_C_ROBOT4 *(mem+50)
#define COM_REFEREE *(mem+51)
#define LOCALIZATION_X *(mem+52)
#define LOCALIZATION_Y *(mem+53)
#define LOCALIZATION_THETA *(mem+54)
#define VISION_MOTOR1_ANGLE *(mem+55)
#define VISION_MOTOR2_ANGLE *(mem+56)
#define VISION_LOST_BALL *(mem+57)
#define VISION_SEARCH_BALL *(mem+58)
#define DECISION_ACTION_VISION *(mem+59)

//Variaveis usadas na simulação do programa TD(lambida)
#define RESET_ROBOT *(mem+60)//Darwin1
#define CAIU_CONT *(mem+61) // é a mesma para ambos os robôs

#define RESET_ROBOT_2 *(mem+62)//Darwin2
#define RESET_ROBOT_3 *(mem+63)//Darwin3

//variaveis double
#define TRANS1 *(memf+1) //Darwin1
#define TRANS2 *(memf+2)
#define TRANS3 *(memf+3)

#define TRANS1_2 *(memf+4)//Darwin2
#define TRANS2_2 *(memf+5)
#define TRANS3_2 *(memf+6)

#define TRANS1_3 *(memf+7)//Darwin3
#define TRANS2_3 *(memf+8)
#define TRANS3_3 *(memf+9)

//----global variables------------------------------------------------
extern int *mem ; //Variável que manipula memória compartilhada
extern float *memf ; //Variável que manipula memória compartilhada

//----Functions prototype---------------------------------------------
int using_shared_memory(); //Função que cria e acopla a memória compartilhada

#endif

