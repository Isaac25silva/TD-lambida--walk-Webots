/*
 * File:          my_controller.c
 * Date:          
 * Description:   
 * Author:        
 * Modifications: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/supervisor.h> //Adicionado
#include <stdio.h>
#include <unistd.h>

#include <blackboard.h>

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "blackboard.h"

//#define DEBUG


int *mem ; //Variável que manipula memória compartilhada
float *memf ; //Variável que manipula memória compartilhada

//Depois de criado a memória compartilhada, para verificar se ela realmente foi criada
// e quantos processos estão utilizando, digite no terminal o comando $ipcs -m
// será a memoria criada ->   key = 0x0000007b    bytes = 2048
// nattch = number of attached processes

int using_shared_memory()
{
    // --- Variaveis usada para memoria compartilhada -----
    int shmid ; // identificador da memoria comum //
    const unsigned short int size = 2048; // tamanho da memória em Bytes
    int flag = 0;
    //-----------------------------------------------------

     // Recuperando ou criando uma memoria compartilhada-------------------
     //

     //shmget:para criar um segmento de memória compartilhada
     if (( shmid = shmget((key_t)KEY, size,0)) == -1)
     {
          perror("Erro no shmget") ;
          printf("\nMemória será criada\n");
         //return(1) ;
        if (( shmid = shmget((key_t)KEY, size, IPC_CREAT|IPC_EXCL|SHM_R|SHM_W)) == -1)
        {
            perror("Erro no shmget") ;
            return(1) ;
        }

     }
#ifdef DEBUG
     printf("Sou o processo com pid: %d \n",getpid()) ;
     printf("Identificador do segmento recuperado: %d \n",shmid) ;
     printf("Este segmento e associado a chave unica: %d\n",(key_t)KEY);
#endif
    //
    // acoplamento do processo a zona de memoria
    //recuperacao do pornteiro sobre a area de memoria comum
    //
    //shmat:retorna um pointeiro para o segmento de memória compartilhada
     if ((mem = (int*)shmat (shmid, 0, flag)) == (int*)-1){
          perror("acoplamento impossivel") ;
          return (2) ;
     }

     memf = (float*)(mem+125);
     //---------------------------------------------------------------------

            /* destruicao do segmento */
            //if ((shmctl(shmid, IPC_RMID, NULL)) == -1){
            // perror("Erro shmctl()");
             // return(1) ;
            //}
    return(0);
}

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv)
{
  //int cont=0;
  
  
  using_shared_memory();

  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  // do this once only Darwin
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Darwin");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  WbFieldRef rot_field = wb_supervisor_node_get_field(robot_node, "rotation");
  int caiu_cont = 0;

  // do this once only Darwin2
  WbNodeRef robot_node_2 = wb_supervisor_node_get_from_def("Darwin2");
  WbFieldRef trans_field_2 = wb_supervisor_node_get_field(robot_node_2, "translation");
  WbFieldRef rot_field_2 = wb_supervisor_node_get_field(robot_node_2, "rotation");

  // do this once only Darwin3
  WbNodeRef robot_node_3 = wb_supervisor_node_get_from_def("Darwin3");
  WbFieldRef trans_field_3 = wb_supervisor_node_get_field(robot_node_3, "translation");
  WbFieldRef rot_field_3 = wb_supervisor_node_get_field(robot_node_3, "rotation");

  while (1) {
    // this is done repeatedly
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
	const double *trans_2 = wb_supervisor_field_get_sf_vec3f(trans_field_2);
	const double *trans_3 = wb_supervisor_field_get_sf_vec3f(trans_field_3);
    TRANS1 = trans[0];
    TRANS2 = trans[1];
    TRANS3 = trans[2];
    TRANS1_2 = trans_2[0];
    TRANS2_2 = trans_2[1];
    TRANS3_2 = trans_2[2];
    TRANS1_3 = trans_3[0];
    TRANS2_3 = trans_3[1];
    TRANS3_3 = trans_3[2];
    //printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);
    wb_robot_step(32);
    
    if(RESET_ROBOT == 1)
    { 
      caiu_cont++;
      CAIU_CONT = caiu_cont;
      // reset robot position
      const double INITIAL[3] = { 0, 0.32004, 0 };
      const double INITIAL_ROT[4] = { 0.211189, 0.971678, -0.106025, 0.944968 };
      wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
      wb_supervisor_field_set_sf_rotation(rot_field, INITIAL_ROT);
      //wb_supervisor_simulation_reset_physics();
      RESET_ROBOT = 0;
      if(caiu_cont > 25)
      {
        caiu_cont = 0;
        wb_supervisor_simulation_revert(); // restart the simulation
        // A medida que o tempo vai passando o robo Darwin vai ficando
        // cada vez mais devagar como se houvesse algum tipo de desgaste fisico
        //entao e necessario restarta a simulaçao
      }
    }

    if(RESET_ROBOT_2 == 1)
    { 
      // reset robot position
      const double INITIAL_2[3] = { 0.2, 0.32004, 4.7 };
      const double INITIAL_ROT_2[4] = { 0.211189, 0.971678, -0.106025, 0.944968 };
      wb_supervisor_field_set_sf_vec3f(trans_field_2, INITIAL_2);
      wb_supervisor_field_set_sf_rotation(rot_field_2, INITIAL_ROT_2);
      //wb_supervisor_simulation_reset_physics();
      RESET_ROBOT_2 = 0;
    }

    if(RESET_ROBOT_3 == 1)
    { 
      // reset robot position
      const double INITIAL_3[3] = { 3.2, 0.32004, -3.7 };
      const double INITIAL_ROT_3[4] = { 0.211189, 0.971678, -0.106025, 0.944968 };
      wb_supervisor_field_set_sf_vec3f(trans_field_3, INITIAL_3);
      wb_supervisor_field_set_sf_rotation(rot_field_3, INITIAL_ROT_3);
      //wb_supervisor_simulation_reset_physics();
      RESET_ROBOT_3 = 0;
    }
    
  }
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
 
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  
  return 0;
}
