#include "SliderJoint.hpp"



namespace mars
{
  namespace ode_physics
  {

    using namespace utils;
    using namespace interfaces;

    SliderJoint::SliderJoint(WorldPhysics *world,
                             data_broker::DataBrokerInterface *dataBroker,
                             configmaps::ConfigMap &config)
        : Joint(world, dataBroker, config)
    {
      Joint::createJoint();
    }

    SliderJoint::~SliderJoint(void)
    {
    }

    Joint *SliderJoint::instantiate(WorldPhysics *world,
                                    data_broker::DataBrokerInterface *dataBroker,
                                    configmaps::ConfigMap &config)
    {
      return new SliderJoint(world, dataBroker, config);
    }

    bool SliderJoint::createJoint(dBodyID body1, dBodyID body2)
    {
      jointId = dJointCreateSlider(theWorld->getWorld(), 0);
      dJointAttach(jointId, body1, body2);
      dJointSetSliderAxis(jointId, config["axis1"]["x"], config["axis1"]["y"],
                         config["axis1"]["z"]);

      if(damping > 0.00000001)
      {
        dJointSetSliderParam(jointId, dParamFMax, damping);
        dJointSetSliderParam(jointId, dParamVel, 0);
      }
      if(spring > 0.00000001)
      {
        dJointSetSliderParam(jointId, dParamLoStop, lo1);
        dJointSetSliderParam(jointId, dParamHiStop, hi1);
        dJointSetSliderParam(jointId, dParamStopCFM, cfm1);
        dJointSetSliderParam(jointId, dParamStopERP, erp1);
      } else if(lo1 != 0)
      {
        dJointSetSliderParam(jointId, dParamLoStop, lo1);
        dJointSetSliderParam(jointId, dParamHiStop, hi1);
      }
      if(jointCFM > 0)
      {
        dJointSetSliderParam(jointId, dParamCFM, jointCFM);
      }
      // good value for the SpaceClimber robot
      // dJointSetSliderParam(jointId, dParamCFM, 0.03);
      // dJointSetSliderParam(jointId, dParamCFM, 0.018);
      // dJointSetSliderParam(jointId, dParamCFM, 0.09);

      return true;
    }

    /// TODO add update state
    void SliderJoint::updateState()
    {
        // todo: - check where we need locking
        //       - handle invert and joint offset mechanism
        //MutexLocker locker(&(theWorld->iMutex));

        position1 = -1*(dReal)dJointGetSliderPosition(jointId);
        //ointGetSliderAxis(jointId, axis);
        axis1_torque.setZero();
        axis2_torque.setZero();
        velocity1 = getVelocity();
        getAxis(&axis1);
        getForce1(&f1);
        getTorque1(&t1);
        motor_torque = feedback.lambda*-1;
    }

    std::vector<std::string> SliderJoint::getEditPattern(const std::string& basePath) const
    {
        std::vector<std::string> result{Joint::getEditPattern(basePath)};
        result.push_back(basePath + "axis/*");
        return result;
    }
  } // end of namespace ode_physics
} // end of namespace mars
