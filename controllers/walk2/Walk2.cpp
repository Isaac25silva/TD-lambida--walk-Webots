/*
When running this controller in the real robot
do not interface via SSh client (i,e, PuTTY)
The result will be on a segmentation fault error.

Instead interface with the robot via remote desktop
(i.e. VNC). Open a terminal window, compile controller
and run.
*/
#include "Walk.hpp"
#include <webots/LED.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <DARwInOPMotionManager.hpp>
#include <DARwInOPGaitManager.hpp>

#include <blackboard.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <random>		/* mt19937, uniform_int_distribution */
#include <chrono>		/* clock from system */
#include <time.h>
#include <iterator>
#include <math.h>       /* pow */

#include <../../config.h>

using namespace webots;
using namespace managers;
using namespace std;

static const char *motorNames[NMOTORS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

Walk::Walk():
    Robot()
{
  mTimeStep = getBasicTimeStep();

  srand((unsigned) time(NULL));
  getLED("HeadLed")->set(0xFF0000);
  getLED("EyeLed")->set(0x00FF00);
  mAccelerometer = getAccelerometer("Accelerometer");
  mAccelerometer->enable(mTimeStep);
  
  getGyro("Gyro")->enable(mTimeStep);
  
  for (int i=0; i<NMOTORS; i++) {
    mMotors[i] = getMotor(motorNames[i]);
    string sensorName = motorNames[i];
    sensorName.push_back('S');
    mPositionSensors[i] = getPositionSensor(sensorName);
    mPositionSensors[i]->enable(mTimeStep);
  }
  
  keyboardEnable(mTimeStep);
  
  mMotionManager = new DARwInOPMotionManager(this);
  mGaitManager = new DARwInOPGaitManager(this, "config.ini");
  robot_fall=0;
  walk_cont = 0;
  negative_reward=Config::NEGATIVE_REWARD;
  lambida=Config::LAMBIDA;
  alpha = Config::ALPHA;
  gamma = Config::GAMMA;
  sigma = Config::SIGMA;
  beta = negative_reward*sigma;
  exploration = Config::EXPLORATION;
  totalReward = 0;
  Episodio = 0;
}

Walk::~Walk() {
}

void Walk::myStep() {
  int ret = step(mTimeStep);
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

void Walk::wait(int ms) {
  double startTime = getTime();
  double s = (double) ms / 1000.0;
  while (s + startTime >= getTime())
    myStep();
}

void Walk::run_best_parameters()
{
cont=0;
numero_de_passos = Config::NUMERO_DE_PASSOS;
num_max_passos = Config::NUM_MAX_PASSOS;
num_max_action = 5;

      cout << "-------Walk example of DARwIn-OP-------" << endl;
      cout << "This example illustrates Gait with Best Parameters" << endl;
      using_shared_memory();
      
      // First step to update sensors values
      myStep();
      
      mMotionManager->playPage(1); // init position
      mMotionManager->playPage(9); // init position
      wait(200);

      //cout<<"   "<<*mGaitManager->ARM_SWING_GAIN_RL<<endl;
      *mGaitManager->arm_swing_gain_RL = 1.0;


      mGaitManager->start();    
      mGaitManager->setXAmplitude(1.0);
      mGaitManager->setYAmplitude(0.0);
      mGaitManager->setAAmplitude(0.0);
      mGaitManager->setBalanceEnable(0);

      *mGaitManager->period_time_RL = 640;
      *mGaitManager->swing_right_left_RL = 34;
      cout<<" period_time = "<<*mGaitManager->period_time_RL;
      cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;

  while(1)
  {
    mGaitManager->start();    
    mGaitManager->setXAmplitude(1.0);
    mGaitManager->setYAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    mGaitManager->setBalanceEnable(0);
    walk_cont=0;
    caiu_cont=0;
    
    updateXYZ();
    RESET_ROBOT_2 = 0;
      
    //Permanece neste while enquanto passos dado for menor que num_max_passos ou caiu_cont<5
    while (true) 
    {
      checkIfFallen();

      cont++;
      
      // if que verifica se foi dado o numero de passos como definido
      if( cont>=((*mGaitManager->period_time_RL/8)*numero_de_passos) )
      {
        cont=0;
        passos++;
        walk_cont++; //Conta quantos passos andou sem cair

        //- Calcula distancia e velocidade----------------------------------------
        speedWalking();
        //------------------------------------------------------------------------
      
        //cout<<"TRANS1_2 "<<TRANS1_2<<" - TRANS2_2 "<<TRANS2_2<<" - TRANS3_2 "<<TRANS3_2<<endl;
        //cout<<*mGaitManager->period_time_RL;
        //*mGaitManager->y_offset_RL;
        //*mGaitManager->z_offset_RL;
        //*mGaitManager->period_time_RL;
        //*mGaitManager->dsp_ratio_RL;
        //*mGaitManager->step_forward_back_ratio_RL;
        //*mGaitManager->foot_height_RL;
        //*mGaitManager->swing_right_left_RL;
        //*mGaitManager->swing_top_down_RL;
        //*mGaitManager->arm_swing_gain_RL;
              
        myStep();
      }
  
      mGaitManager->step(mTimeStep);
      
      // step
      myStep(); 
      if(walk_cont>num_max_passos || caiu_cont>10)
        break;
    }
   }

       
}

//====================================================================
// function containing the main feedback loop
void Walk::run2P(bool newFile) {

  cont=0;
  numero_de_passos = Config::NUMERO_DE_PASSOS;
  num_max_passos = Config::NUM_MAX_PASSOS;
  num_max_action = 5;
  std::mt19937 mt_rand;
  // obtain a seed from the system clock:
  mt_rand.seed( std::chrono::system_clock::now().time_since_epoch().count() );
  // Naõ utilizar seed como time(NULL) porque ele puxa o time do webots.

  init_RL(); //start the RL's variable
  std::uniform_int_distribution<> RandStateP1(0, p1_value.size()-1);
  std::uniform_int_distribution<> RandStateP2(0, p2_value.size()-1);
  ReinforcementLearning RL(p2_value.size(), p1_value.size(), num_max_action);
  RL.EXPLORATION_THRESHOLD = exploration;
  RL.ALPHA = alpha;
  RL.GAMMA = gamma;
  cout << "-------Walk example of DARwIn-OP-------" << endl;
  cout << "This example illustrates Gait Manager" << endl;

  using_shared_memory();
  
  //--------------------Inicie o programa abrindo o arquivo ou aleatorio---------------------------------
  if(newFile == 0)
      OpenFile2P(RL.qValue2p, RL.NUM_ACTION);

  else
  {
      RL.init_qvalues(RL.qValue2p, p2_value.size(), p1_value.size());
      passos=0;
      cout<<"Criando um novo Arquivo"<<endl;
      saveQ2p(RL.qValue2p, "Qvalue.dat");
      
      //---Episodio novo------------------
      std::fstream File;
      File.open("./dados/Episodio.dat", std::ios::out);
      File << "#Recompensa por Episodio\n";
      File.flush();
      File.close();
      //-----------------------------------

      //---Episodio novo------------------
      std::fstream File1;
      File1.open("./dados/Error.dat", std::ios::out);
      File1 << "#Erro de V por Episodio\n";
      File1.flush();
      File1.close();
      //-----------------------------------
  }

  initZ2p();

  // First step to update sensors values
  myStep();
 
  mMotionManager->playPage(1); // init position
  mMotionManager->playPage(9); // init position
  wait(200);
 
  //cout<<"   "<<*mGaitManager->ARM_SWING_GAIN_RL<<endl;
  *mGaitManager->arm_swing_gain_RL = 1.0;
 
  while(1)
  {
    mGaitManager->start();    
    mGaitManager->setXAmplitude(1.5);
    mGaitManager->setYAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    mGaitManager->setBalanceEnable(0);
    walk_cont=0;
    caiu_cont=0;
    //---- generate the random states -------------------------------
    //do
    //{
        state_p1 = RandStateP1(mt_rand);
        state_p2 = RandStateP2(mt_rand);
        //wait(200);
        //cout<<"V = "<<RL.qValue2p[state_p2][state_p1]  <<" beta = "<< beta << endl;
    //}while(RL.qValue2p[state_p2][state_p1] < beta);
    //---------------------------------------------------------------
    
    //---Zera o rastro de elegibilidade Z(s,a)-----------------------
    zeroZ2pValue();
    //---------------------------------------------------------------

	//--------- Guarda o valor de V ---------------------------------
	V_old = RL.qValue2p;
	//---------------------------------------------------------------
    
    totalReward = 0; //zera a soma de recompensas por episodio
    
    //--------------------------------------------------------------------
    *mGaitManager->period_time_RL = p1_value[state_p1];
    *mGaitManager->swing_right_left_RL = p2_value[state_p2];
    cout<<" period_time = "<<*mGaitManager->period_time_RL;
    cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;
    //-------------------------------------------------------------------
    updateXYZ();
    RESET_ROBOT_2 = 0;
      
    //Permanece neste while enquanto passos dado for menor que num_max_passos ou caiu_cont<5
    while (true) 
    {
      checkIfFallen();
	  if(CAIU_CONT==24)
		saveQ2p(RL.qValue2p, "Qvalue.dat");
		

      cont++;
      
      // if que verifica se foi dado o numero de passos como definido
      if( cont>=((*mGaitManager->period_time_RL/8)*numero_de_passos) )
      {
        cont=0;
        passos++;
        walk_cont++; //Conta quantos passos andou sem cair
        
        //A ← action given by π for S
        acao = RL.choose_best_action_egreedy(RL.qValue2p, state_p2, state_p1);
        //acao = RL.best_qvalue_action(RL.qValue2p, state_p2, state_p1);
        
        cout<<"action = "<<acao<<" | Num passos sem cair = "<<walk_cont<<endl;
        new_state_p1 = state_p1;
        new_state_p2 = state_p2;
        
        //Observa o novo estado (S') a partir da açao A----------
        //--------acao incrementa-----------
        if(acao == 0)
          if(new_state_p1<(p1_value.size()-1))
             new_state_p1++;
        if(acao == 1)
          if(new_state_p2<(p2_value.size()-1))
             new_state_p2++;
        //----------------------------------
        //--------acao decrementa-----------
        if(acao == 2)
          if(new_state_p1>0)
             new_state_p1--;
        if(acao == 3)
          if(new_state_p2>0)
             new_state_p2--;
        //----------------------------------
        //-------permanece com a mesma acao se acao for 4 --

        //- Calcula distancia e velocidade----------------------------------------
        speedWalking();
        //------------------------------------------------------------------------
        
        //-------Reward-------------------------------
        giveReward();
        //--------------------------------------------
      
        sumReward();
      
        //Escolhendo a acao A' do estado S'-------------------------------
        //acao_futura = RL.choose_best_action_egreedy(RL.qValue2p, new_state_p2, new_state_p1);
        
        //A ∗ ← arg max a Q(S , a)----------------------------------------
        //acao_otima = RL.best_qvalue_action(RL.qValue2p, new_state_p2, new_state_p1);
      
        //δ ← r + γQ(S',A∗) − Q(S, A)-------------------------------------
        delta_erro = reward + gamma*(RL.qValue2p[new_state_p2][new_state_p1]) - RL.qValue2p[state_p2][state_p1];
      
        //Z(S, A) ← Z(S, A) + 1-------------------------------------------
        Z2p[state_p2][state_p1] = Z2p[state_p2][state_p1]+1;
        
        //---para todos os estados s e todas as ações a(s) faça-----------------------
        for (unsigned int i = 0; i < p2_value.size(); i++)
           for (unsigned int j = 0; j < p1_value.size(); j++)
              {
                  //Q(s, a) ← Q(s, a) + αδZ(s, a)-------------------------
                  RL.qValue2p[i][j] = RL.qValue2p[i][j] + alpha*delta_erro*Z2p[i][j];
                  
                  //se A' == A∗ então------------------------------
                  //Z(s, a) ← γλZ(s, a)
                  //fim
                  //if(acao_futura == acao_otima)
                  Z2p[i][j] = gamma*lambida*Z2p[i][j];
                  //else
                      //Z2p[i][j][k] = 0;
              
              }
        //----------------------------------------------------------------------------
      
        //cout<<"TRANS1_2 "<<TRANS1_2<<" - TRANS2_2 "<<TRANS2_2<<" - TRANS3_2 "<<TRANS3_2<<endl;
        //cout<<*mGaitManager->period_time_RL;
        //*mGaitManager->y_offset_RL;
        //*mGaitManager->z_offset_RL;
        //*mGaitManager->period_time_RL;
        //*mGaitManager->dsp_ratio_RL;
        //*mGaitManager->step_forward_back_ratio_RL;
        //*mGaitManager->foot_height_RL;
        //*mGaitManager->swing_right_left_RL;
        //*mGaitManager->swing_top_down_RL;
        //*mGaitManager->arm_swing_gain_RL;
              
        myStep();
        
        
        //---Update the Q value------------------------------------------------------------
        //best_new_qval = RL.best_qvalue(RL.qValue2p, new_state_p2, new_state_p1);
  
        //qval = RL.qValue2p[state_p2][state_p1][acao];
  	
        //RL.qValue2p[state_p2][state_p1][acao] = (1 - ALPHA)*qval + ALPHA*(reward + GAMMA*best_new_qval);
        //-----------------------------------------------------------------------------------
        
        if(passos%100 == 0)
        {
           //std::vector<double> qValue;
           //changeMatrix3DtoVector(RL.qValue2p, qValue);
           //char* fileName = "Qvalue.dat";
           saveQ2p(RL.qValue2p, "Qvalue.dat");
           if(passos%500 == 0)
           {
               saveQexcel2p(RL.qValue2p, "./backup/backupTable2p.dat");
               saveQexcel2pZ(RL.qValue2p, "./backup/backupMatriz2p.dat");
           }

           if(passos%1000 == 0)
           {
			   if(passos == 7000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/7Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/7Matriz2p.dat");
			   }
			   if(passos == 15000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/15Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/15Matriz2p.dat");
			   }
			   if(passos == 50000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/50Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/50Matriz2p.dat");
			   }
			   if(passos == 100000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/100Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/100Matriz2p.dat");
			   }
			   if(passos == 200000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/200Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/200Matriz2p.dat");
			   }
			   if(passos == 260000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/260Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/260Matriz2p.dat");
			   }
			   if(passos == 300000)
			   {
				   saveQexcel2p(RL.qValue2p, "./dados/300Table2p.dat");
				   saveQexcel2pZ(RL.qValue2p, "./dados/300Matriz2p.dat");
			   }
            }
        }
        
        //-----Update the state---------------------------------
        state_p1 = new_state_p1;
        state_p2 = new_state_p2;
        *mGaitManager->period_time_RL = p1_value[state_p1];
        *mGaitManager->swing_right_left_RL = p2_value[state_p2];
        cout<<" period_time = "<<*mGaitManager->period_time_RL;
        cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;
        //--------------------------------------------------------
      }
  
      mGaitManager->step(mTimeStep);
      
      // step
      myStep(); 
      //Verifica se finalizou o episódio-------------------------------
	  if(walk_cont>num_max_passos || caiu_cont>10) 
      {
        saveEpisodios(Episodio); // salva os valores da recompensa acumulada
		Sum_error = 0;
        for (unsigned int i = 0; i < p2_value.size(); i++)
           for (unsigned int j = 0; j < p1_value.size(); j++)
				Sum_error += pow(V_old[i][j] - RL.qValue2p[i][j], 2);

		saveError(Episodio); // salva os valores do erro
        Episodio++;
        break; // sai do while para iniciar um novo episódio
      }
	  //---------------------------------------------------------------
    }
    
    if(passos>1000002)
        return;  
    
   }
}
//====================================================================
//----------------------
void Walk::run3P(bool newFile) {

cont=0;
numero_de_passos = 4;
num_max_passos = 40;
num_max_action = 7;

  init_RL(); //start the RL's variable
  ReinforcementLearning RL3p(p3_value.size(), p2_value.size(), p1_value.size(), num_max_action);
  RL3p.EXPLORATION_THRESHOLD = exploration;
  RL3p.ALPHA = alpha;
  RL3p.GAMMA = gamma;
  cout << "-------Walk example of DARwIn-OP-------" << endl;
  cout << "This example illustrates Gait Manager" << endl;

  using_shared_memory();
  
  //--------------------Inicie o programa abrindo o arquivo ou aleatorio---------------------------------
  if(newFile == 0)
  {
      OpenFile3P(RL3p.qValue3p, num_max_action);
  }
  else
  {
      RL3p.init_qvalues(RL3p.qValue3p, p3_value.size(),p2_value.size(), p1_value.size());
      passos=0;
      //cout<<"Criando um novo Arquivo"<<endl;
      //std::vector<double> qValue;
      //changeMatrix4DtoVector(RL3p.qValue3p, qValue);
      //char* fileName = "Qvalue3p.dat";
      saveQ3p(RL3p.qValue3p);
  }

  //-----------Inicia o tamanho do Z3p-----------
  initZ3p();
  //---------------------------------------------

  // First step to update sensors values
  myStep();
 
  mMotionManager->playPage(1); // init position
  mMotionManager->playPage(9); // init position
  wait(200);

  //cout<<"   "<<*mGaitManager->ARM_SWING_GAIN_RL<<endl;
  *mGaitManager->arm_swing_gain_RL = 1.0;

  while(1)
  {
    mGaitManager->start();    
    mGaitManager->setXAmplitude(1.5);
    mGaitManager->setYAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    mGaitManager->setBalanceEnable(0);
    walk_cont=0;
    caiu_cont=0;
    
    //---- generate the random states -------------------------------
    initRandomState3p();
    //---------------------------------------------------------------

    //---Zera o rastro de elegibilidade Z(s,a)-----------------------
    zeroZ3pValue();
    //---------------------------------------------------------------
    
    //--------------------------------------------------------------------
    *mGaitManager->period_time_RL = p1_value[state_p1];
    *mGaitManager->swing_right_left_RL = p2_value[state_p2];
    *mGaitManager->swing_top_down_RL = p3_value[state_p3];
    cout<<" period_time = "<<*mGaitManager->period_time_RL;
    cout<<" | swing_top_down = "<<*mGaitManager->swing_top_down_RL;
    cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;
    //-------------------------------------------------------------------
    updateXYZ(); //Atualiza os valores da posiçao Global inicial do robo
    RESET_ROBOT_2 = 0;
     
    //Permanece neste while enquanto passos dado for menor que num_max_passos ou caiu_cont<5
    while (true) 
    {
      checkIfFallen();
 
      cont++;
      
      // if que verifica se foi dado o numero de passos como definido
      if( cont>=((*mGaitManager->period_time_RL/8)*numero_de_passos) )
      {
        cont=0;
        passos++;
        walk_cont++; //Conta quantos passos andou sem cair
        
        //A ← action given by π for S
        acao = RL3p.choose_best_action_egreedy(RL3p.qValue3p, state_p3, state_p2, state_p1);
        //acao = RL3p.best_qvalue_action(RL3p.qValue3p, state_p3, state_p2, state_p1);
        
        //acao = RL3p.best_qvalue_action(RL3p.qValue3p, state_p3, state_p2, state_p1);
        cout<<"action = "<<acao<<" | Num passos sem cair = "<<walk_cont<<endl;
        new_state_p1 = state_p1;
        new_state_p2 = state_p2;
        new_state_p3 = state_p3;
    
        //Observa o novo estado (S') a partir da açao A--------
        //--------acao incrementa-----------
        if(acao == 0)
          if(new_state_p1<(p1_value.size()-1))
             new_state_p1++;
        if(acao == 1)
          if(new_state_p2<(p2_value.size()-1))
             new_state_p2++;
        if(acao == 2)
          if(new_state_p3<(p3_value.size()-1))
             new_state_p3++;
        //----------------------------------
        //--------acao decrementa-----------
        if(acao == 3)
          if(new_state_p1>0)
             new_state_p1--;
        if(acao == 4)
          if(new_state_p2>0)
             new_state_p2--;
        if(acao == 5)
          if(new_state_p3>0)
             new_state_p3--;
        //----------------------------------
        //-------permanece com a mesma acao se acao for 6 --

        //- Calcula distancia e velocidade----------------------------------------
        speedWalking();
        //------------------------------------------------------------------------
        
        //-------Reward-------------------------------
        giveReward();
        //--------------------------------------------

        //Escolhendo a acao A' do estado S'-------------------------------
        //acao_futura = RL3p.choose_best_action_egreedy(RL3p.qValue3p, new_state_p3, new_state_p2, new_state_p1);
        
        //A ∗ ← arg max a Q(S',a)----------------------------------------
        //acao_otima = RL3p.best_qvalue_action(RL3p.qValue3p, new_state_p3, new_state_p2, new_state_p1);
      
        //δ ← r + γV(S') − V(S)-------------------------------------
        delta_erro = reward + gamma*(RL3p.qValue3p[new_state_p3][new_state_p2][new_state_p1]) - RL3p.qValue3p[state_p3][state_p2][state_p1];
      
        //Z(S) ← Z(S) + 1-------------------------------------------
        Z3p[state_p3][state_p2][state_p1] = Z3p[state_p3][state_p2][state_p1]+1;
        
        //---para todos os estados s e todas as ações a(s) faça-----------------------
        for (unsigned int w = 0; w < p3_value.size(); w++)
            for (unsigned int i = 0; i < p2_value.size(); i++)
               for (unsigned int j = 0; j < p1_value.size(); j++)
                  {
                      //V(s) ← V(s) + αδZ(s)-------------------------
                      RL3p.qValue3p[w][i][j] = RL3p.qValue3p[w][i][j] + alpha*delta_erro*Z3p[w][i][j];
                      
                      //se A' == A∗ então------------------------------
                      //Z(s, a) ← γλZ(s, a)
                      //fim
                      //if(acao_futura == acao_otima)
                          Z3p[w][i][j] = gamma*lambida*Z3p[w][i][j];
                     // else
                          //Z3p[w][i][j][k] = 0;
                  
                  }
        //----------------------------------------------------------------------------
      
        myStep();

        
        if(passos%10 == 0)
        {
           //std::vector<double> qValue;
           //changeMatrix4DtoVector(RL3p.qValue3p, qValue);
           //char* fileName = "Qvalue3p.dat";
           saveQ3p(RL3p.qValue3p);
           if(passos%200 == 0)
               saveQexcel3p(RL3p.qValue3p);
        }
        
        //-----Update the state---------------------------------
        state_p1 = new_state_p1;
        state_p2 = new_state_p2;
        state_p3 = new_state_p3;
        //acao = acao_futura;
        *mGaitManager->period_time_RL = p1_value[state_p1];
        *mGaitManager->swing_right_left_RL = p2_value[state_p2];
        *mGaitManager->swing_top_down_RL = p3_value[state_p3];
        cout<<" period_time = "<<*mGaitManager->period_time_RL;
        cout<<" | swing_top_down = "<<*mGaitManager->swing_top_down_RL;
        cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;
        //--------------------------------------------------------
      }
  
      mGaitManager->step(mTimeStep);
      
      // step
      myStep(); 
      if(walk_cont>num_max_passos || caiu_cont>10)
        break;
      
    }
   }
}
//====================================================================
//----------------------
void Walk::run4P(bool newFile) {

cont=0;
numero_de_passos = 4;
num_max_passos = 40;
num_max_action = 9;

  init_RL(); //start the RL's variable
  ReinforcementLearning RL4p(p4_value.size(), p3_value.size(), p2_value.size(), p1_value.size(), num_max_action);
  RL4p.EXPLORATION_THRESHOLD = exploration;
  RL4p.ALPHA = alpha;
  RL4p.GAMMA = gamma;
  cout << "-------Walk example of DARwIn-OP-------" << endl;
  cout << "This example illustrates Gait Manager" << endl;

  using_shared_memory();
  
  //--------------------Inicie o programa abrindo o arquivo ou aleatorio---------------------------------
  if(newFile == 0)
  {
      OpenFile4P(RL4p.qValue4p, num_max_action);
  }
  else
  {
      RL4p.init_qvalues(RL4p.qValue4p, p4_value.size(), p3_value.size(),p2_value.size(), p1_value.size());
      passos=0;
      saveQ4p(RL4p.qValue4p);
  }

  //-----------Inicia o tamanho do Z3p-----------
  initZ4p();
  //---------------------------------------------

  // First step to update sensors values
  myStep();
 
  mMotionManager->playPage(1); // init position
  mMotionManager->playPage(9); // init position
  wait(200);

  //cout<<"   "<<*mGaitManager->ARM_SWING_GAIN_RL<<endl;
  *mGaitManager->arm_swing_gain_RL = 1.0;

  while(1)
  {
    mGaitManager->start();    
    mGaitManager->setXAmplitude(1.5);
    mGaitManager->setYAmplitude(0.0);
    mGaitManager->setAAmplitude(0.0);
    mGaitManager->setBalanceEnable(0);
    walk_cont=0;
    caiu_cont=0;
    
    //---- generate the random states -------------------------------
    initRandomState4p();
    //---------------------------------------------------------------

    //---Zera o rastro de elegibilidade Z(s,a)-----------------------
    zeroZ4pValue();
    //---------------------------------------------------------------
    
    //--------------------------------------------------------------------
    *mGaitManager->period_time_RL = p1_value[state_p1];
    *mGaitManager->swing_right_left_RL = p2_value[state_p2];
    *mGaitManager->swing_top_down_RL = p3_value[state_p3];
    *mGaitManager->foot_height_RL = p4_value[state_p4];
    cout<<" period_time = "<<*mGaitManager->period_time_RL;
    cout<<" | swing_top_down = "<<*mGaitManager->swing_top_down_RL;
    cout<<" | foot_height = "<<*mGaitManager->foot_height_RL;
    cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;
    //-------------------------------------------------------------------
    updateXYZ(); //Atualiza os valores da posiçao Global inicial do robo
    RESET_ROBOT_2 = 0;
     
    //Permanece neste while enquanto passos dado for menor que num_max_passos ou caiu_cont<5
    while (true) 
    {
      checkIfFallen();
 
      cont++;
      
      // if que verifica se foi dado o numero de passos como definido
      if( cont>=((*mGaitManager->period_time_RL/8)*numero_de_passos) )
      {
        cont=0;
        passos++;
        walk_cont++; //Conta quantos passos andou sem cair

        //A ← action given by π for S
        acao = RL4p.choose_best_action_egreedy(RL4p.qValue4p, state_p4, state_p3, state_p2, state_p1); //com e-greedy
       // acao = RL4p.best_qvalue_action(RL4p.qValue4p, state_p4, state_p3, state_p2, state_p1);      //sem e-greedy
       
        cout<<"action = "<<acao<<" | Num passos sem cair = "<<walk_cont<<endl;
        new_state_p1 = state_p1;
        new_state_p2 = state_p2;
        new_state_p3 = state_p3;
        new_state_p4 = state_p4;
    
        //Observa o novo estado (S') a partir da açao A--------
        //--------acao incrementa-----------
        if(acao == 0)
          if(new_state_p1<(p1_value.size()-1))
             new_state_p1++;
        if(acao == 1)
          if(new_state_p2<(p2_value.size()-1))
             new_state_p2++;
        if(acao == 2)
          if(new_state_p3<(p3_value.size()-1))
             new_state_p3++;
        if(acao == 3)
          if(new_state_p4<(p4_value.size()-1))
             new_state_p4++;
        //----------------------------------
        //--------acao decrementa-----------
        if(acao == 4)
          if(new_state_p1>0)
             new_state_p1--;
        if(acao == 5)
          if(new_state_p2>0)
             new_state_p2--;
        if(acao == 6)
          if(new_state_p3>0)
             new_state_p3--;
        if(acao == 7)
          if(new_state_p4>0)
             new_state_p4--;
        //----------------------------------
        //-------permanece com a mesma acao se acao for 8 --

        //- Calcula distancia e velocidade----------------------------------------
        speedWalking();
        //------------------------------------------------------------------------
        
        //-------Reward-------------------------------
        giveReward();
        //--------------------------------------------

        //Escolhendo a acao A' do estado S'-------------------------------
        //acao_futura = RL4p.choose_best_action_egreedy(RL4p.qValue4p, new_state_p4, new_state_p3, new_state_p2, new_state_p1);
        
        //A ∗ ← arg max a Q(S',a)----------------------------------------
        //acao_otima = RL4p.best_qvalue_action(RL4p.qValue4p, new_state_p4, new_state_p3, new_state_p2, new_state_p1);
      
        //δ ← r + γQ(S',A∗) − Q(S, A)-------------------------------------
        delta_erro = reward + gamma*(RL4p.qValue4p[new_state_p4][new_state_p3][new_state_p2][new_state_p1]) - 
        RL4p.qValue4p[state_p4][state_p3][state_p2][state_p1];
      
        //Z(S, A) ← Z(S, A) + 1-------------------------------------------
        Z4p[state_p4][state_p3][state_p2][state_p1] = Z4p[state_p4][state_p3][state_p2][state_p1]+1;
        
        //---para todos os estados s e todas as ações a(s) faça-----------------------
        for (unsigned int x = 0; x < p4_value.size(); x++)
            for (unsigned int w = 0; w < p3_value.size(); w++)
                for (unsigned int i = 0; i < p2_value.size(); i++)
                   for (unsigned int j = 0; j < p1_value.size(); j++)
                      {
                          //Q(s, a) ← Q(s, a) + αδZ(s, a)-------------------------
                          RL4p.qValue4p[x][w][i][j] = RL4p.qValue4p[x][w][i][j] + alpha*delta_erro*Z4p[x][w][i][j];
                          
                          //se A' == A∗ então------------------------------
                          //Z(s, a) ← γλZ(s, a)
                          //fim
                         // if(acao_futura == acao_otima)
                          Z4p[x][w][i][j] = gamma*lambida*Z4p[x][w][i][j];
                          //else
                              //Z4p[x][w][i][j] = 0;
                      
                      }
        //----------------------------------------------------------------------------
      
        myStep();
      
        if(passos%10 == 0)
        {
           //std::vector<double> qValue;
           //changeMatrix4DtoVector(RL3p.qValue3p, qValue);
           //char* fileName = "Qvalue3p.dat";
           saveQ4p(RL4p.qValue4p);
           if(passos%200 == 0)
               saveQexcel4p(RL4p.qValue4p);
        }
        
        //-----Update the state---------------------------------
        state_p1 = new_state_p1;
        state_p2 = new_state_p2;
        state_p3 = new_state_p3;
        state_p4 = new_state_p4;
        *mGaitManager->period_time_RL = p1_value[state_p1];
        *mGaitManager->swing_right_left_RL = p2_value[state_p2];
        *mGaitManager->swing_top_down_RL = p3_value[state_p3];
        *mGaitManager->foot_height_RL = p4_value[state_p4];
        cout<<" period_time = "<<*mGaitManager->period_time_RL;
        cout<<" | swing_top_down = "<<*mGaitManager->swing_top_down_RL;
        cout<<" | foot_height = "<<*mGaitManager->foot_height_RL;
        cout<<" | swing_right_left = "<<*mGaitManager->swing_right_left_RL<<endl;
        //--------------------------------------------------------
      }
  
      mGaitManager->step(mTimeStep);
      
      // step
      myStep(); 
      if(walk_cont>num_max_passos || caiu_cont>10)
        break;
      
    }
   }
}
//====================================================================
//====================================================================
//====================================================================
//====================================================================
//------------
void Walk::checkIfFallen() 
{
  if(TRANS2_2<0.15)
  {
      getLED("EyeLed")->set(0xFF0000);
      caiu_cont++;
      //cout<<"Robo Caiu "<<endl;
      //mMotionManager->playPage(1); // init position9
      mMotionManager->playPage(9); // init position
      RESET_ROBOT_2 = 1;
      cont=1000; //restart the counter and penalized
      //stateP1 =state_p1;
      //stateP2 = state_p2;
      walk_cont=0;
      robot_fall=1;
      wait(2);
      updateXYZ(); //Atualiza os valores da posiçao Global inicial do robo
      getLED("EyeLed")->set(0x00FF00);
  }
  else
      robot_fall=0;
}

//====================================================================
//------------
void Walk::init_RL()
{
    //----Variavel period_time ----------------------------------------
    double utmost_period_time = 960;
    double least_period_time = 450;
    double step_period_time = 15;
    int max_state_period_time = (utmost_period_time - least_period_time)/step_period_time + 1;
    cout<<"Number states period_time = "<<max_state_period_time<<endl;
    for(int x=0; x<max_state_period_time; x++)
    {
        p1_value.push_back(least_period_time+step_period_time*x);
        cout<<" "<<p1_value[x];
    }
    cout<<endl;
    //-----------------------------------------------------------------

    //----Variavel swing_right_left -----------------------------------
    double utmost_right_left = 40;
    double least_right_left = 15;
    double step_right_left = 2;
    int max_state_right_left = (utmost_right_left - least_right_left)/step_right_left + 1;
    cout<<"Number states swing_right_left = "<<max_state_right_left<<endl;
    for(int x=0; x<max_state_right_left; x++)
    {
        p2_value.push_back(least_right_left+step_right_left*x);
        cout<<" "<<p2_value[x];
    }
    cout<<endl;
    //------------------------------------------------------------------
    
    //----Variavel swing_top_down_RL -----------------------------------
    double utmost_swing_top_down = 10;
    double least_swing_top_down = 2;
    double step_swing_top_down = 1;
    int max_state_swing_top_down = (utmost_swing_top_down - least_swing_top_down)/step_swing_top_down + 1;
    cout<<"Number states swing_top_down = "<<max_state_swing_top_down<<endl;
    for(int x=0; x<max_state_swing_top_down; x++)
    {
        p3_value.push_back(least_swing_top_down+step_swing_top_down*x);
        cout<<" "<<p3_value[x];
    }
    cout<<endl;
    //-------------------------------------------------------------------
    
    //----Variavel foot_hight_RL -----------------------------------
    double utmost_foot_height = 30;
    double least_foot_height = 10;
    double step_foot_height = 2;
    int max_state_foot_height = (utmost_foot_height - least_foot_height)/step_foot_height + 1;
    cout<<"Number states foot_height = "<<max_state_foot_height<<endl;
    for(int x=0; x<max_state_foot_height; x++)
    {
        p4_value.push_back(least_foot_height+step_foot_height*x);
        cout<<" "<<p4_value[x];
    }
    cout<<endl;
    //-------------------------------------------------------------------
    
}
//==============================================================================================
//--------------------Salva a tabel Q no arquivo------------------------------------------------
void Walk::saveQ2p(std::vector<std::vector<double> >  Value2p, std::string fileName)
{
    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;

    const char *fileName1;
    //std::string fileName = "Qvalue.dat";
    fileName1 = fileName.c_str();

    File.open(fileName1, std::ios::out);
    if (File.good() && File.is_open())
    {
         for (unsigned int i = 0; i < p2_value.size(); ++i)
            for (unsigned int j = 0; j < p1_value.size(); ++j)
                    File << Value2p[i][j] << separator;
        File << passos << separator;
        File << Episodio << separator;
        File << std::endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//==============================================================================================
//--------------------Salva a tabel Q no arquivo------------------------------------------------
void Walk::saveQ3p(std::vector<std::vector<std::vector<double> > > qValue3p)
{
    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;

    File.open("Qvalue3p.dat", std::ios::out);
    if (File.good() && File.is_open())
    {
      for (unsigned int x = 0; x < p3_value.size(); ++x)
         for (unsigned int i = 0; i < p2_value.size(); ++i)
            for (unsigned int j = 0; j < p1_value.size(); ++j)
                    File << qValue3p[x][i][j] << separator;
        File << passos << separator;
        File << Episodio << separator;
        File << std::endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}

//==============================================================================================
//--------------------Salva a tabel Q no arquivo------------------------------------------------
void Walk::saveQ4p(std::vector<std::vector<std::vector<std::vector<double> > > > qValue4p)
{
    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;

    File.open("Qvalue4p.dat", std::ios::out);
    if (File.good() && File.is_open())
    {
      for (unsigned int y = 0; y < p4_value.size(); ++y)
        for (unsigned int x = 0; x < p3_value.size(); ++x)
           for (unsigned int i = 0; i < p2_value.size(); ++i)
              for (unsigned int j = 0; j < p1_value.size(); ++j)
                      File << qValue4p[y][x][i][j] << separator;
      File << passos << separator;
      File << Episodio << separator;
      File << std::endl;
      File.flush();
      File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}


//==============================================================================================
//--------------------Salva a tabel Q para abrir no excel ---------------------------------------
void Walk::saveQexcel2p(std::vector<std::vector<double> > Value2p, std::string fileName)
{
    cout<<"Salva a tabel Q para abrir no excel "<<endl;

    const char *fileName1;
    fileName1 = fileName.c_str(); 
   
    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;
    
    File.open(fileName1, std::ios::out);
    File << "#Passos_total= " << passos <<endl;
    File << "#Episodio= " << Episodio <<endl;
    File << "#Negative_reward= " << negative_reward <<endl;
    File << "#Num_de_passos_por_passo= " << numero_de_passos <<endl;
    File << "#Num_max_passos_por_episodio= "<<num_max_passos<<endl;
    File << "#Lambida= "<<lambida<<endl;
    File << "#Alpha= "<<alpha<<endl;
    File << "#Gamma= "<<gamma<<endl;
    File << "#Exploraçao= " << exploration <<endl<<endl;
    File << "#p2_value " << "p1_value " << "Qvalue2p " <<endl;
    if (File.good() && File.is_open())
    {
         for (unsigned int i = 0; i < p2_value.size(); i++)
           for (unsigned int j = 0; j < p1_value.size(); j++)
                  File << p2_value[i]<<" "<< p1_value[j]<<" " << " "<< Value2p[i][j] <<endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//==============================================================================================
//--------------------Salva a tabel Q para abrir no excel ---------------------------------------
void Walk::saveQexcel2pZ(std::vector<std::vector<double> > Value2p, std::string fileName)
{
    cout<<"Salva a tabel Q para abrir no excel "<<endl;

    const char *fileName1;
    fileName1 = fileName.c_str();

    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;
    
    File.open(fileName1, std::ios::out);
    File << "#Passos_total= " << passos <<endl;
    File << "#Episodio= " << Episodio <<endl;
    File << "#Negative_reward= " << negative_reward <<endl;
    File << "#Num_de_passos_por_passo= " << numero_de_passos <<endl;
    File << "#Num_max_passos_por_episodio= "<<num_max_passos<<endl;
    File << "#Lambida= "<<lambida<<endl;
    File << "#Alpha= "<<alpha<<endl;
    File << "#Gamma= "<<gamma<<endl;
	File << "#Beta= "<<beta<<endl;
    File << "#Exploraçao= " << exploration <<endl<<endl;
    //File << "p2_value " << "p1_value " << "Qvalue2p " <<endl;
    if (File.good() && File.is_open())
    {
         File <<"#x = [";
         for (unsigned int j = 0; j < p1_value.size(); j++)
             File << p1_value[j]<<" ";
         File << "];" <<endl;

         File <<"#y = [";
         for (unsigned int j = 0; j < p2_value.size(); j++)
             File << p2_value[j]<<" ";
         File << "];" <<endl; 
         File <<"#z = [\n";
         for (unsigned int i = 0; i < p2_value.size(); i++)
         {
           //File << p2_value[i]<<" ";
           for (unsigned int j = 0; j < p1_value.size(); j++)
                  File << Value2p[i][j] <<" ";
           File << endl;
         }
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//==============================================================================================
//--------------------Salva a tabel Q para abrir no excel ---------------------------------------
void Walk::saveQexcel3p(std::vector<std::vector<std::vector<double> > > Value3p)
{
    cout<<"Salva a tabel Q para abrir no excel "<<endl;

    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;
    
    File.open("excelQvalue3p.dat", std::ios::out);
    File << "passoss" << passos <<endl;
    File << "p3_value " << "p2_value " << "p1_value " << "Qvalue3p" <<endl;
    if (File.good() && File.is_open())
    {
       for (unsigned int w = 0; w < p3_value.size(); w++)
         for (unsigned int i = 0; i < p2_value.size(); i++)
           for (unsigned int j = 0; j < p1_value.size(); j++)
                  File << p3_value[w]<<" "<< p2_value[i]<<" "<< p1_value[j]<<" " << " "<< Value3p[w][i][j] <<endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}

//==============================================================================================
//--------------------Salva a tabel Q para abrir no excel ---------------------------------------
void Walk::saveQexcel4p(std::vector<std::vector<std::vector<std::vector<double> > > > Value4p)
{
    cout<<"Salva a tabel Q para abrir no excel "<<endl;

    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;
    
    File.open("excelQvalue4p.dat", std::ios::out);
    File << "passoss" << passos <<endl;
    File << "p4_value "<< "p3_value " << "p2_value " << "p1_value " << "Qvalue4p" <<endl;
    if (File.good() && File.is_open())
    {
       for (unsigned int x = 0; x < p4_value.size(); x++)
         for (unsigned int w = 0; w < p3_value.size(); w++)
           for (unsigned int i = 0; i < p2_value.size(); i++)
             for (unsigned int j = 0; j < p1_value.size(); j++)
                    File << p4_value[x] <<" "<< p3_value[w]<<" "<< p2_value[i]<<" "<< p1_value[j]<<" " <<" "<< Value4p[x][w][i][j] <<endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//==============================================================================================
//--------------------Salva o numero de passos por Episodio-------------------------------------
void Walk::saveEpisodios(unsigned int Episodio)
{
    std::fstream File;

    File.open("./dados/Episodio.dat", std::ios::app | std::ios::out);
    if (File.good() && File.is_open())
    {
        File << Episodio <<" "<< totalReward;
        File << std::endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//==============================================================================================
//--------------------Salva o erro do V por Episodio-------------------------------------
void Walk::saveError(unsigned int Episodio)
{
    std::fstream File;

    File.open("./dados/Error.dat", std::ios::app | std::ios::out);
    if (File.good() && File.is_open())
    {
        File << Episodio <<" "<< Sum_error;
        File << std::endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}


//===============================================================================================
//-------------Abre o arquivo que contem --------------------------------------------------------
void Walk::openFiletoGetQvalueVector(std::vector<double> &qValue, std::string fileName)
{
    const char *fileName1;
    //std::string fileName = "Qvalue.dat";
    fileName1 = fileName.c_str();

    std::ifstream File(fileName1);
    std::istream_iterator<float> start(File), end;
    std::vector<float> vectorTemp(start, end);
    for(unsigned int x=0; x < vectorTemp.size(); x++)
      qValue.push_back(vectorTemp[x]);

}

//=================================================================================================================
/*
void Walk::changeMatrix3DtoVector(std::vector<std::vector<std::vector<double> > > qValue2p, std::vector<double> &qValue)
{
         for (unsigned int i = 0; i < p2_value.size(); ++i)
            for (unsigned int j = 0; j < p1_value.size(); ++j)
                for (unsigned int k = 0; k < num_max_action; ++k)
                    qValue.push_back(qValue2p[i][j][k]);
}

void Walk::changeMatrix4DtoVector(std::vector<std::vector<std::vector<std::vector<double> > > > qValue3p, std::vector<double> &qValue)
{
      for (unsigned int x = 0; x < p3_value.size(); ++x)
         for (unsigned int i = 0; i < p2_value.size(); ++i)
            for (unsigned int j = 0; j < p1_value.size(); ++j)
                for (unsigned int k = 0; k < num_max_action; ++k)
                    qValue.push_back(qValue3p[x][i][j][k]);
}
*/
void Walk::OpenFile2P(std::vector<std::vector<double> > &Value2p, unsigned int NUM_ACTION)
{
      std::vector<double> qValue;
      unsigned  int inc=0;
      openFiletoGetQvalueVector(qValue, "Qvalue.dat");//abre o arquivo salvo e passa os valores para Qvalue
      for (unsigned int i = 0; i < p2_value.size(); i++)
         for (unsigned int j = 0; j < p1_value.size(); j++)
             {
                 Value2p[i][j] = qValue[inc];
                 inc++;
             }
      passos = qValue[inc];
      Episodio = qValue[inc+1];
      cout<<"Open File | passos = "<<passos<<endl;
}

void Walk::OpenFile3P(std::vector<std::vector<std::vector<double> > > &Value3p, unsigned int NUM_ACTION)
{
      std::vector<double> qValue;
      unsigned  int inc=0;
      openFiletoGetQvalueVector(qValue, "Qvalue3p.dat");//abre o arquivo salvo e passa os valores para Qvalue
      for (unsigned int w = 0; w < p3_value.size(); w++)
        for (unsigned int i = 0; i < p2_value.size(); i++)
           for (unsigned int j = 0; j < p1_value.size(); j++)
               {
                   Value3p[w][i][j] = qValue[inc];
                   inc++;
               }
      passos = qValue[inc];
      Episodio = qValue[inc+1];
      cout<<"Open File | passos = "<<passos<<endl;
}

void Walk::OpenFile4P(std::vector<std::vector<std::vector<std::vector<double> > > > &Value4p, unsigned int NUM_ACTION)
{
      std::vector<double> qValue;
      unsigned  int inc=0;
      openFiletoGetQvalueVector(qValue, "Qvalue4p.dat");//abre o arquivo salvo e passa os valores para Qvalue
      for (unsigned int x = 0; x < p4_value.size(); x++)
        for (unsigned int w = 0; w < p3_value.size(); w++)
          for (unsigned int i = 0; i < p2_value.size(); i++)
             for (unsigned int j = 0; j < p1_value.size(); j++)
                 {
                     Value4p[x][w][i][j] = qValue[inc];
                     inc++;
                 }
      passos = qValue[inc];
      Episodio = qValue[inc+1];
      cout<<"Open File | passos = "<<passos<<endl;
}

void Walk::initRandomState()
{
    //---- generate the random states -------------------------------
    state_p2 = (rand()/(float)(RAND_MAX))*(float)(p2_value.size()-1) ;// % 100)/(float)(100/p1_value.size());
    state_p1 = (rand()/(float)(RAND_MAX))*(float)(p1_value.size()-1);//% 100)/(float)(100/p2_value.size());
    //cout<<"state_p1 = "<<state_p1<<" | state_p2 = "<<state_p2<<" | acao = "<<acao<<endl;
    //std::mt19937 mt_rand(time(0));
    //std::uniform_int_distribution<> disP1(0, p1_value.size());
    //std::uniform_int_distribution<> disP2(0, p2_value.size());
    //---------------------------------------------------------------
}

void Walk::initRandomState3p()
{
    //---- generate the random states -------------------------------
    state_p3 = (rand()/(float)(RAND_MAX))*(float)(p3_value.size()-1) ;
    state_p2 = (rand()/(float)(RAND_MAX))*(float)(p2_value.size()-1) ;// % 100)/(float)(100/p1_value.size());
    state_p1 = (rand()/(float)(RAND_MAX))*(float)(p1_value.size()-1);//% 100)/(float)(100/p2_value.size());
    cout<<"state_p1 = "<<state_p1<<" | state_p2 = "<<state_p2<<" | state_p3 = "<<state_p3<<" | acao = "<<acao<<endl;
}

void Walk::initRandomState4p()
{
    //---- generate the random states -------------------------------
    state_p4 = (rand()/(float)(RAND_MAX))*(float)(p4_value.size()-1) ;
    state_p3 = (rand()/(float)(RAND_MAX))*(float)(p3_value.size()-1) ;
    state_p2 = (rand()/(float)(RAND_MAX))*(float)(p2_value.size()-1) ;// % 100)/(float)(100/p1_value.size());
    state_p1 = (rand()/(float)(RAND_MAX))*(float)(p1_value.size()-1);//% 100)/(float)(100/p2_value.size());
    cout<<"state_p1 = "<<state_p1<<" | state_p2 = "<<state_p2<<" | state_p3 = "<<state_p3<<" | state_p4 = "<<state_p4<<" | acao = "<<acao<<endl;
}

void Walk::initZ2p()
{
    Z2p.resize(p2_value.size());
    for (unsigned int j = 0; j < p2_value.size(); ++j)
        Z2p[j].resize(p1_value.size());
}

void Walk::initZ3p()
{
      Z3p.resize(p3_value.size());
      for (unsigned int j = 0; j < p3_value.size(); ++j) 
      {
          Z3p[j].resize(p2_value.size());
          for (unsigned int k = 0; k < p2_value.size(); ++k)
              Z3p[j][k].resize(p1_value.size());
      }
}

void Walk::initZ4p()
{
      Z4p.resize(p4_value.size());
      for (unsigned int i = 0; i < p4_value.size(); ++i)
      {
          Z4p[i].resize(p3_value.size());
          for (unsigned int j = 0; j < p3_value.size(); ++j) 
          {
              Z4p[i][j].resize(p2_value.size());
              for (unsigned int k = 0; k < p2_value.size(); ++k)
                  Z4p[i][j][k].resize(p1_value.size());
          }
      }
}

void Walk::zeroZ2pValue()
{
    //---Zera o rastro de elegibilidade Z(s,a)-----------------------
    for (unsigned int i = 0; i < p2_value.size(); i++)
       for (unsigned int j = 0; j < p1_value.size(); j++)
              Z2p[i][j] = 0;
    //---------------------------------------------------------------
}

void Walk::zeroZ3pValue()
{
    //---Zera o rastro de elegibilidade Z(s,a)-----------------------
    for (unsigned int w = 0; w < p3_value.size(); w++)
      for (unsigned int i = 0; i < p2_value.size(); i++)
         for (unsigned int j = 0; j < p1_value.size(); j++)
                Z3p[w][i][j] = 0;
    //---------------------------------------------------------------
}

void Walk::zeroZ4pValue()
{
    //---Zera o rastro de elegibilidade Z(s,a)-----------------------
    for (unsigned int y = 0; y < p4_value.size(); y++)
      for (unsigned int w = 0; w < p3_value.size(); w++)
        for (unsigned int i = 0; i < p2_value.size(); i++)
           for (unsigned int j = 0; j < p1_value.size(); j++)
                  Z4p[y][w][i][j] = 0;
    //---------------------------------------------------------------
}

void Walk::speedWalking()
{
        //- Calcula distancia e velocidade----------------------------------------
        distancia = sqrt(pow(TRANS1_2-transIni1,2)+pow(TRANS3_2-transIni3,2));
        if(distancia>0.01 && robot_fall==0)
        {
          velocidade = (distancia*100)/((numero_de_passos*(*mGaitManager->period_time_RL))/1000);
          cout<<"\e[1;94mDistancia percorrida = "<<distancia<<" | Velocidade = "<<velocidade<<"\e[0m"<<endl;
        }
        else
        {
            velocidade = 0;
            cout<<"\e[1;31mDistancia percorrida = "<<distancia<<" | Velocidade = "<<velocidade<<" Robo Caiu\e[0m"<<endl;
        }
        transIni1 = TRANS1_2;
        transIni2 = TRANS2_2;
        transIni3 = TRANS3_2;
        //-------------------------------------------------------------------
}

void Walk::giveReward()
{
      if(robot_fall==1)
           reward = negative_reward;
      else
	  {
		if(velocidade<30)
           reward = pow(10,(velocidade+0.00001)/10)/3;
		else
		   reward = 50;
	  }
}

void Walk::sumReward()
{
    totalReward += reward;
}

void Walk::updateXYZ()
{
    wait(5);
    transIni1 = TRANS1_2;
    transIni2 = TRANS2_2;
    transIni3 = TRANS3_2;
}
