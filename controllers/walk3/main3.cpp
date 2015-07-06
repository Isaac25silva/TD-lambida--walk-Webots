// File:          main.cpp
// Date:          January 2013
// Description:   Manage the entree point function
// Author:        david.mansolino@epfl.ch

#include "Walk.hpp"


#include <cstdlib>

using namespace webots;

int main(int argc, char **argv)
{

  Walk *controller = new Walk();
  //controller->run_best_parameters();
  controller->run2P(0); // 1 inicia nova tabela e 0 usa valores salvos no arquivo
  //controller->run3P(0);
  //controller->run4P(0);
  delete controller;
  return EXIT_FAILURE;
}

