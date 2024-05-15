#include "FixedJoint.hpp"


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;

        FixedJoint::FixedJoint(WorldPhysics *world,
                               data_broker::DataBrokerInterface *dataBroker,
                               configmaps::ConfigMap &config)
            : Joint(world, dataBroker, config)
        {
            Joint::createJoint();
        }

        FixedJoint::~FixedJoint(void)
        {
        }

        Joint* FixedJoint::instantiate(WorldPhysics *world,
                                       data_broker::DataBrokerInterface *dataBroker,
                                       configmaps::ConfigMap &config)
        {
            return new FixedJoint(world, dataBroker, config);
        }

        bool FixedJoint::createJoint(dBodyID body1, dBodyID body2)
        {
            if(body1 || body2)
            {
                jointId = dJointCreateFixed(theWorld->getWorld(), 0);
                dJointAttach(jointId, body1, body2);
                dJointSetFixed(jointId);
                // used for the integration study of the SpaceClimber
                //dJointSetFixedParam(jointId, dParamCFM, cfm1);//0.0002);
                //dJointSetFixedParam(jointId, dParamERP, erp1);//0.0002);
                //dJointSetFixedParam(jointId, dParamCFM, 0.001);
            } else
            {
                return 0;
            }
            return 1;
        }

        void FixedJoint::updateState()
        {
            // todo: - check were we need locking
            //       - handle invert and joint offset mechanism
            //MutexLocker locker(&(theWorld->iMutex));

            getForce1(&f1);
            getTorque1(&t1);
        }
    } // end of namespace ode_physics
} // end of namespace mars
