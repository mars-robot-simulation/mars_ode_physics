/*
 *  Copyright 2022, DFKI GmbH Robotics Innovation Center
 *
 *  This file is part of the MARS simulation framework.
/**
 * \file Hinge2Joint.hpp
 * \author Malte Langosz and Team
 * \brief Uses https://www.ode.org/wiki/index.php?title=Manual#Hinge-2
 */

#pragma once

#include <mars_utils/MutexLocker.h>
#include "Joint.hpp"
#include <string>

namespace mars
{
    namespace ode_physics
    {
        class Hinge2Joint : public Joint
        {
        public:
            Hinge2Joint(WorldPhysics *world,
                       data_broker::DataBrokerInterface *dataBroker,
                       configmaps::ConfigMap &config);
            virtual ~Hinge2Joint(void);
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

