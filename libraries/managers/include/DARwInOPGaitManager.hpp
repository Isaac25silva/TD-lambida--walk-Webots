// File:          DARwInOPGaitManager.hpp
// Date:          20th of September 2011
// Description:   Facade between webots and the darwin-op framework
//                allowing to handle the gait generator
// Author:        fabien.rohrer@cyberbotics.com

#ifndef DARWINOP_GAIT_MANAGER_HPP
#define DARWINOP_GAIT_MANAGER_HPP

#include <string>

#define DGM_NSERVOS 20
#define DGM_BOUND(x,a,b) (((x)<(a))?(a):((x)>(b))?(b):(x))

namespace webots {
  class Robot;
  class Motor;
}

namespace Robot {
  class Walking;
}

namespace managers {
  using namespace Robot;
  class DARwInOPGaitManager {
    public:
                       DARwInOPGaitManager(webots::Robot *robot, const std::string &iniFilename);
      virtual         ~DARwInOPGaitManager();
      bool             isCorrectlyInitialized() { return mCorrectlyInitialized; }
      
      void             setXAmplitude(double x) { mXAmplitude = DGM_BOUND(x, -1.5, 1.5) * 20.0; } 
      void             setYAmplitude(double y) { mYAmplitude = DGM_BOUND(y, -1.0, 1.0) * 40.0; }
      void             setAAmplitude(double a) { mAAmplitude = DGM_BOUND(a, -1.0, 1.0) * 50.0; }
      void             setMoveAimOn(bool q) { mMoveAimOn = q; }
      void             setBalanceEnable(bool q) { mBalanceEnable = q; }

      void             start();
      void             step(int ms);
      void             stop();
      Walking         *mWalking;
 
     //--- Variaveis de manipulaçao do RL-----------------    
      double *y_offset_RL;
      double *z_offset_RL;
      double *period_time_RL;
      double *dsp_ratio_RL;
      double *step_forward_back_ratio_RL;
      double *foot_height_RL;
      double *swing_right_left_RL;
      double *swing_top_down_RL;
      double *arm_swing_gain_RL;
		
     //---------------------------------------------------

    private:
      webots::Robot   *mRobot;
      bool             mCorrectlyInitialized;
      int              mBasicTimeStep;
      double           mXAmplitude;
      double           mAAmplitude;
      double           mYAmplitude;
      bool             mMoveAimOn;
      bool             mBalanceEnable;
      bool             mIsWalking;

#ifndef CROSSCOMPILATION
      void             myStep();
      double           valueToPosition(unsigned short value);
      webots::Motor   *mMotors[DGM_NSERVOS];
#endif
  };
}

#endif
