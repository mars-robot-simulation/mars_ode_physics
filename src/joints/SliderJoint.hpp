 /**
 * \file SliderJoint.hpp
 * \author Malte Langosz and Team
 * \brief Uses https://www.ode.org/wiki/index.php?title=Manual#Slider
 */

#pragma once

#include <mars_utils/MutexLocker.h>
#include "Joint.hpp"
#include <string>


namespace mars
{
    namespace ode_physics
    {

        class SliderJoint : public Joint
        {
        public:
            SliderJoint(WorldPhysics *world,
                       data_broker::DataBrokerInterface *dataBroker,
                       configmaps::ConfigMap &config);
            virtual ~SliderJoint(void);
            static Joint* instantiate(WorldPhysics *world,
                                      data_broker::DataBrokerInterface *dataBroker,
                                      configmaps::ConfigMap &config);
            virtual bool createJoint(dBodyID body1, dBodyID body2) override;

            // --- mars::interfaces::ConfigMapInterface ---
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
        protected:
            void updateState() override;

        };

    } // end of namespace ode_physics
} // end of namespace mars
