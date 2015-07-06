/*******************************************************************************************************/
/* File:         PositionSensor.hpp                                                                    */
/* Date:         July 14                                                                               */
/* Description:  Wrapper of the PositionSensor Webots API for the DARwIn-OP real robot                 */
/* Author:       david.mansolino@epfl.ch                                                               */
/* Copyright (c) 2014 Cyberbotics - www.cyberbotics.com                                                */
/*******************************************************************************************************/

#ifndef POSITION_SENSOR_HPP
#define POSITION_SENSOR_HPP

#include <webots/Robot.hpp>
#include <webots/Device.hpp>
#include <map>

namespace webots {
  class PositionSensor: public Device  {
    public:
      enum {
        ROTATIONAL = 0
      };

                    PositionSensor(const std::string &name); //Use Robot::getMotor() instead
      virtual      ~PositionSensor();

      virtual void  enable(int ms);
      virtual void  disable();
      int           getSamplingPeriod() const;
      double        getValue() const;

      int           getType() const;
    private:
      static void   initStaticMap();
      static std::map<const std::string, int> mNamesToIDs;
      static std::map<const std::string, int> mNamesToInitPos;
      
      void          setPresentPosition(int position);

      // For Bulk Read //
      int           mPresentPosition;
      
      int           mFeedback;

      friend int Robot::step(int ms);
      friend     Robot::Robot();
  };
}

#endif //POSITION_SENSOR_HPP
