
#ifndef QL_H
#define QL_H

#include <vector>		/* vector */
//#include <time.h>       /* time */
//#include <stdlib.h>     /* srand, rand */

class ReinforcementLearning
{
	public:
   		/*! Construtor. */
    	ReinforcementLearning(unsigned int tamP2, unsigned int tamP1, unsigned int tamAcao);
    	ReinforcementLearning(unsigned int tamP3, unsigned int tamP2, unsigned int tamP1, unsigned int tamAcao);
    	ReinforcementLearning(unsigned int tamP4, unsigned int tamP3, unsigned int tamP2, unsigned int tamP1, unsigned int tamAcao);

    	/*! Atributo utilizado para guardar valores em matrix. */
		std::vector<std::vector<double> > qValue2p;
		std::vector<std::vector<std::vector<double> > > qValue3p;
		std::vector<std::vector<std::vector<std::vector<double> > > > qValue4p;

		void init_qvalues(std::vector<std::vector<double> > &qValue2p, int num_p2, int num_p1);
		void init_qvalues(std::vector<std::vector<std::vector<double> > > &qValue3p, int num_p3, int num_p2, int num_p1);
		void init_qvalues(std::vector<std::vector<std::vector<std::vector<double> > > > &qValue4p, int num_p4, int num_p3, int num_p2, int num_p1);

		double best_qvalue(std::vector<std::vector<double> > &qValue2p, int state_p2, int state_p1);
		double best_qvalue(std::vector<std::vector<std::vector<double> > > &qValue3p, int state_p3, int state_p2, int state_p1);
		double best_qvalue(std::vector<std::vector<std::vector<std::vector<double> > > > &qValue4p, int state_p4, int state_p3, int state_p2, int state_p1);

		int best_qvalue_action(std::vector<std::vector<double> > &qValue2p, int state_p2, int state_p1);
		int best_qvalue_action(std::vector<std::vector<std::vector<double> > > &qValue3p, int state_p3, int state_p2, int state_p1);
		int best_qvalue_action(std::vector<std::vector<std::vector<std::vector<double> > > > &qValue4p, int state_p4, int state_p3, int state_p2, int state_p1);

		int choose_best_action_egreedy(std::vector<std::vector<double> > &qValue2p, int state_p2, int state_p1);
		int choose_best_action_egreedy(std::vector<std::vector<std::vector<double> > > &qValue3p, int state_p3, int state_p2, int state_p1);	
		int choose_best_action_egreedy(std::vector<std::vector<std::vector<std::vector<double> > > > &qValue4p, int state_p4, int state_p3, int state_p2, int state_p1);	


		unsigned int NUM_ACTION;
		int p4_size;
		int p3_size;
		int p2_size;
		int p1_size;
		double GAMMA;
		double ALPHA;
		double EXPLORATION_THRESHOLD;
	private:

		int act;
};

#endif
