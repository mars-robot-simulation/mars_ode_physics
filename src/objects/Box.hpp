/**
 * \file Box.hpp
 * \author Malte Langosz and Team
 *
 */

#pragma once

#include "Frame.hpp"
#include "Object.hpp"
#include <string>

namespace mars
{
    namespace ode_physics
    {
        class Box : public Object
        {
        public:
            Box(std::shared_ptr<Frame> frame, configmaps::ConfigMap &config);
            virtual ~Box(void);
            static Object *instantiate(std::shared_ptr<Frame> frame, configmaps::ConfigMap &config);
            virtual bool createMass() override;

        private:
            configmaps::ConfigMap config;
        };

    } // end of namespace ode_physics
} // end of namespace mars
