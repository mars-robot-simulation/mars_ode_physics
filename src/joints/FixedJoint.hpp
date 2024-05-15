 /**
 * \file FixedJoint.hpp
 * \author Malte Langosz and Team
 * \brief Uses https://www.ode.org/wiki/index.php?title=Manual#Fixed
 */

#pragma once

#include <mars_utils/MutexLocker.h>
#include "Joint.hpp"
#include <string>

namespace mars
{
    namespace ode_physics
    {

        class FixedJoint : public Joint
        {
        public:
            FixedJoint(WorldPhysics *world,
                       data_broker::DataBrokerInterface *dataBroker,
                       configmaps::ConfigMap &config);
            virtual ~FixedJoint(void);
            static Joint* instantiate(WorldPhysics *world,
                                      data_broker::DataBrokerInterface *dataBroker,
                                      configmaps::ConfigMap &config);
            virtual bool createJoint(dBodyID body1, dBodyID body2) override;
        protected:
            void updateState() override;
        };

    } // end of namespace ode_physics
} // end of namespace mars
