
#include "Hinge2Joint.hpp"

#include <mars_utils/mathUtils.h>  // vectorToConfigMap


namespace mars
{
    namespace ode_physics
    {
        using namespace utils;
        using namespace interfaces;

        Hinge2Joint::Hinge2Joint(WorldPhysics *world,
                                 data_broker::DataBrokerInterface *dataBroker,
                                 configmaps::ConfigMap &config)
            : Joint(world, dataBroker, config)
        {
            Joint::createJoint();
        }

        Hinge2Joint::~Hinge2Joint(void)
        {
        }

        Joint *Hinge2Joint::instantiate(WorldPhysics *world,
                                        data_broker::DataBrokerInterface *dataBroker,
                                        configmaps::ConfigMap &config)
        {
            return new Hinge2Joint(world, dataBroker, config);
        }

        bool Hinge2Joint::createJoint(dBodyID body1, dBodyID body2)
        {
            if(body1 || body2)
            {
                jointId = dJointCreateHinge2(theWorld->getWorld(), 0);
                dJointAttach(jointId, body1, body2);
                dJointSetHinge2Anchor(jointId, config["anchor"]["x"], config["anchor"]["y"],
                                      config["anchor"]["z"]);
                dJointSetHinge2Axis1(jointId, config["axis1"]["x"], config["axis1"]["y"],
                                     config["axis1"]["z"]);
                dJointSetHinge2Axis2(jointId, config["axis2"]["x"], config["axis2"]["y"],
                                     config["axis2"]["z"]);

                if(damping > 0.00000001)
                {
                    dJointSetHinge2Param(jointId, dParamFMax, damping);
                    dJointSetHinge2Param(jointId, dParamVel, 0);
                    dJointSetHinge2Param(jointId, dParamFMax2, damping);
                    dJointSetHinge2Param(jointId, dParamVel2, 0);
                }
                if(spring > 0.00000001)
                {
                    dJointSetHinge2Param(jointId, dParamLoStop, lo1);
                    dJointSetHinge2Param(jointId, dParamHiStop, hi1);
                    dJointSetHinge2Param(jointId, dParamLoStop, lo2);
                    dJointSetHinge2Param(jointId, dParamHiStop, hi2);
                    dJointSetHinge2Param(jointId, dParamStopCFM, cfm1);
                    dJointSetHinge2Param(jointId, dParamStopERP, erp1);
                    dJointSetHinge2Param(jointId, dParamStopCFM, cfm2);
                    dJointSetHinge2Param(jointId, dParamStopERP, erp2);
                } else if(lo1 != 0 && lo2 != 0)
                {
                    dJointSetHinge2Param(jointId, dParamLoStop, lo1);
                    dJointSetHinge2Param(jointId, dParamHiStop, hi1);
                    dJointSetHinge2Param(jointId, dParamLoStop, lo2);
                    dJointSetHinge2Param(jointId, dParamHiStop, hi2);
                }
                if(jointCFM > 0)
                {
                    dJointSetHinge2Param(jointId, dParamCFM, jointCFM);
                }

                return true;
            }
        }

        void Hinge2Joint::updateState()
        {
            //TODO

        }

        configmaps::ConfigMap Hinge2Joint::getConfigMap() const
        {
            configmaps::ConfigMap result = Joint::getConfigMap();

            mars::utils::Vector axis2;
            getAxis2(&axis2);
            result["axis2"] = mars::utils::vectorToConfigItem(axis2);

            return result;
        }

        std::vector<std::string> Hinge2Joint::getEditPattern(const std::string& basePath) const
        {
            std::vector<std::string> result{Joint::getEditPattern(basePath)};
            result.push_back(basePath + "axis/*");
            result.push_back(basePath + "axis2/*");
            result.push_back(basePath + "anchor/*");
            return result;
        }
    } // end of namespace ode_physics
} // end of namespace mars
