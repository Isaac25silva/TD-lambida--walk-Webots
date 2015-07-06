// File:          Walk.hpp
// Date:          January 2013
// Description:   Example showing how to use the gait manager
//                and keyboard inputs
// Author:        david.mansolino@epfl.ch

#ifndef WALK_HPP
#define WALK_HPP

#define NMOTORS 20

#include <webots/Robot.hpp>
#include <vector>
#include "ql.h"

namespace managers {
  class DARwInOPMotionManager;
  class DARwInOPGaitManager;
}

namespace webots {
  class Motor;
  class PositionSensor;
  class LED;
  class Camera;
  class Accelerometer;
  class Gyro;
  class Speaker;
};

class Walk : public webots::Robot {
  public:
                                     Walk();
    virtual                         ~Walk();
    void                             run2P(bool newFile);
    void                             run3P(bool newFile);
    void                             run4P(bool newFile);
    void                             run_best_parameters();   
    void                             checkIfFallen();

  private:
    int                              mTimeStep;
    double transIni1;
    double transIni2;
    double transIni3;
    bool robot_fall;
    double totalReward;
    unsigned int walk_cont;
    unsigned int caiu_cont;
    unsigned int cont;
    unsigned int state_p1;
    unsigned int state_p2;
    unsigned int state_p3;
    unsigned int state_p4;
    unsigned int new_state_p1;
    unsigned int new_state_p2;
    unsigned int new_state_p3;
    unsigned int new_state_p4;
    unsigned int acao;
    unsigned int acao_otima;
    unsigned int acao_futura;
    double delta_erro;
    double lambida;
    double alpha;
    double gamma;
    double beta;
    double sigma;
    double exploration;
    double reward;
    double best_new_qval;
    double qval;
    unsigned int Episodio;
    unsigned int passos;
    double distancia;
    double velocidade;
    unsigned int numero_de_passos;
    unsigned int num_max_passos;
    unsigned int num_max_action;
    double       negative_reward;
	double		 Sum_error;
    /*! Atributo utilizado para guardar valores em vetores. */
    std::vector<double> p1_value;
    std::vector<double> p2_value;
    std::vector<double> p3_value;
    std::vector<double> p4_value;
    /*! Atributo utilizado para guardar valores em matrix. */
    std::vector<std::vector<double> > Z2p;
    std::vector<std::vector<std::vector<double> > > Z3p;
    std::vector<std::vector<std::vector<std::vector<double> > > > Z4p;
	/*! Atributo utilizado para guardar valores em matrix do V anterior. */
	std::vector<std::vector<double> > V_old;
    
    void                             myStep();
    void                             wait(int ms);
    void                             init_RL();
    //void saveQ(std::vector<double> qValue, char* fileName);
    void openFiletoGetQvalueVector(std::vector<double> &qValue, std::string fileName);
    void saveQ2p(std::vector<std::vector<double> > Value2p, std::string fileName);
    void saveQ3p(std::vector<std::vector<std::vector<double> > > qValue3p);
    void saveQ4p(std::vector<std::vector<std::vector<std::vector<double> > > > qValue4p);
    //void changeMatrix3DtoVector(std::vector<std::vector<std::vector<double> > > qValue2p, std::vector<double> &qValue);
    //void changeMatrix4DtoVector(std::vector<std::vector<std::vector<std::vector<double> > > > qValue3p, std::vector<double> &qValue);
    void OpenFile2P(std::vector<std::vector<double> > &Value2p, unsigned int NUM_ACTION);
    void OpenFile3P(std::vector<std::vector<std::vector<double> > > &Value3p, unsigned int NUM_ACTION);
    void OpenFile4P(std::vector<std::vector<std::vector<std::vector<double> > > > &Value3p, unsigned int NUM_ACTION);
    void initRandomState();
    void initRandomState3p();
    void initRandomState4p();
    void initZ2p();
    void initZ3p();
    void initZ4p();
    void zeroZ2pValue();
    void zeroZ3pValue();
    void zeroZ4pValue();
    void speedWalking();
    void giveReward();
    void sumReward();
    void saveEpisodios(unsigned int Episodio);
	void saveError(unsigned int Episodio);
    void saveQexcel2p(std::vector<std::vector<double> > Value2p, std::string fileName);
    void saveQexcel2pZ(std::vector<std::vector<double> > Value2p, std::string fileName);
    void saveQexcel3p(std::vector<std::vector<std::vector<double> > > Value3p);
    void saveQexcel4p(std::vector<std::vector<std::vector<std::vector<double> > > > Value4p);
    void updateXYZ();
    
    webots::Motor                   *mMotors[NMOTORS];
    webots::PositionSensor *mPositionSensors[NMOTORS];
    webots::Accelerometer           *mAccelerometer;
    
    managers::DARwInOPMotionManager *mMotionManager;
    managers::DARwInOPGaitManager   *mGaitManager;
};

#endif
