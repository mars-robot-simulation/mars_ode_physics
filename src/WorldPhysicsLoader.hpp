/**
 * \file WorldPhysicsLoader.hpp
 * \author Malte Langosz
 * \brief "WorldPhysicsLoader" factory class to generate new physics instances.
 *
 */

#pragma once

#include "WorldPhysics.hpp"

#include <lib_manager/LibInterface.hpp>

namespace mars
{
    namespace ode_physics
    {

        class WorldPhysicsLoader : public lib_manager::LibInterface
        {
        public:
            explicit WorldPhysicsLoader(lib_manager::LibManager *theManager);
            virtual ~WorldPhysicsLoader(void);

            // LibInterface methods
            int getLibVersion() const
            {
                return 1;
            }

            const std::string getLibName() const
            {
                return std::string("mars_ode_physics");
            }

            CREATE_MODULE_INFO();

            std::shared_ptr<interfaces::PhysicsInterface> createWorldInstance();
        };

    } // end of namespace ode_physics
} // end of namespace mars
