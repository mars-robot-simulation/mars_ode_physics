 /**
 * \file UniversalJoint.hpp
 * \author Malte Langosz and Team
 * \brief Uses https://www.ode.org/wiki/index.php?title=Manual#Universal
 */

#pragma once

#include <mars_utils/MutexLocker.h>
#include "Joint.hpp"
#include <string>


namespace mars
{
    namespace ode_physics
    {

        class UniversalJoint : public Joint
        {
        public:
            UniversalJoint(WorldPhysics *world,
                       data_broker::DataBrokerInterface *dataBroker,
                       configmaps::ConfigMap &config);
            virtual ~UniversalJoint(void);
            static Joint* instantiate(WorldPhysics *world,
                                      data_broker::DataBrokerInterface *dataBroker,
                                      configmaps::ConfigMap &config);
            virtual bool createJoint(dBodyID body1, dBodyID body2) override;

            // --- mars::interfaces::ConfigMapInterface ---
            configmaps::ConfigMap getConfigMap() const override;
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
        protected:
            void updateState() override;

        };

    } // end of namespace ode_physics
} // end of namespace mars
