 /**
 * \file JointFactory.hpp
* \author Malte Langosz and Team
 * \brief "ODEJointFactory" creates ode joints.
 *
 */

#pragma once

#include "joints/Joint.hpp"

namespace mars
{
    namespace ode_physics
    {

        using namespace ::mars::interfaces;

        typedef Joint* (*instantiateJointfPtr)(WorldPhysics *world,
                                               data_broker::DataBrokerInterface *dataBroker,
                                               configmaps::ConfigMap &config);

        class JointFactory
        {
        public:
            static JointFactory& Instance();
            std::shared_ptr<Joint> createJoint(WorldPhysics *world,
                                               data_broker::DataBrokerInterface *dataBroker,
                                               configmaps::ConfigMap &config);
            void addJointType(const std::string& type, instantiateJointfPtr funcPtr);
            const std::map<const std::string, instantiateJointfPtr>& getAvailableJoints() const noexcept
            {
                return availableJoints;
            }
        protected:
            JointFactory();
            virtual ~JointFactory();  
            std::map<const std::string, instantiateJointfPtr> availableJoints;
  
        };

    }
}
