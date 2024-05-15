
#include "UniversalJoint.hpp"

#include <mars_utils/mathUtils.h>  // vectorToConfigMap


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;

        UniversalJoint::UniversalJoint(WorldPhysics *world,
                                 data_broker::DataBrokerInterface *dataBroker,
                                 configmaps::ConfigMap &config)
            : Joint(world, dataBroker, config)
        {
            Joint::createJoint();
        }

        UniversalJoint::~UniversalJoint(void)
        {
        }

        Joint *UniversalJoint::instantiate(WorldPhysics *world,
                                        data_broker::DataBrokerInterface *dataBroker,
                                        configmaps::ConfigMap &config)
        {
            return new UniversalJoint(world, dataBroker, config);
        }

        bool UniversalJoint::createJoint(dBodyID body1, dBodyID body2)
        {
            if (body1 || body2)
            {

                jointId = dJointCreateHinge2(theWorld->getWorld(), 0);
                dJointAttach(jointId, body1, body2);
                dJointSetHinge2Anchor(jointId, config["anchor"]["x"], config["anchor"]["y"],
                                      config["anchor"]["z"]);
                dJointSetHinge2Axis1(jointId, config["axis1"]["x"], config["axis1"]["y"],
                                     config["axis1"]["z"]);
                dJointSetHinge2Axis2(jointId, config["axis2"]["x"], config["axis2"]["y"],
                                     config["axis2"]["z"]);

                if (damping > 0.00000001)
                {
                    dJointSetUniversalParam(jointId, dParamFMax, damping);
                    dJointSetUniversalParam(jointId, dParamVel, 0);
                    dJointSetUniversalParam(jointId, dParamFMax2, damping);
                    dJointSetUniversalParam(jointId, dParamVel2, 0);
                }
                if (spring > 0.00000001)
                {
                    dJointSetUniversalParam(jointId, dParamLoStop, lo1);
                    dJointSetUniversalParam(jointId, dParamHiStop, hi1);
                    dJointSetUniversalParam(jointId, dParamLoStop, lo2);
                    dJointSetUniversalParam(jointId, dParamHiStop, hi2);
                    dJointSetUniversalParam(jointId, dParamStopCFM, cfm1);
                    dJointSetUniversalParam(jointId, dParamStopERP, erp1);
                    dJointSetUniversalParam(jointId, dParamStopCFM, cfm2);
                    dJointSetUniversalParam(jointId, dParamStopERP, erp2);
                }
                else if (lo1 != 0 && lo2 != 0)
                {
                    dJointSetUniversalParam(jointId, dParamLoStop, lo1);
                    dJointSetUniversalParam(jointId, dParamHiStop, hi1);
                    dJointSetUniversalParam(jointId, dParamLoStop, lo2);
                    dJointSetUniversalParam(jointId, dParamHiStop, hi2);
                }
                if (jointCFM > 0)
                {
                    dJointSetUniversalParam(jointId, dParamCFM, jointCFM);
                }

                return true;
            }
        }

        void UniversalJoint::updateState()
        {
            //TODO
            std::cout << "not implemented yet" << std::endl;
        }
    
        configmaps::ConfigMap UniversalJoint::getConfigMap() const
        {
            configmaps::ConfigMap result = Joint::getConfigMap();

            mars::utils::Vector axis2;
            getAxis2(&axis2);
            result["axis2"] = mars::utils::vectorToConfigItem(axis2);

            return result;
        }

        std::vector<std::string> UniversalJoint::getEditPattern(const std::string& basePath) const
        {
            std::vector<std::string> result{Joint::getEditPattern(basePath)};
            result.push_back(basePath + "axis/*");
            result.push_back(basePath + "axis2/*");
            result.push_back(basePath + "anchor/*");
            return result;
        }
    } // end of namespace ode_physics
} // end of namespace mars
