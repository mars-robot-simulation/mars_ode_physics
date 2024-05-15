 /**
 * \file ObjectFactory.hpp
* \author Malte Langosz and Team
 * \brief "ObjectFactory" creates physical objects.
 *
 */

#pragma once

#include "Object.hpp"

namespace mars
{
    namespace ode_physics
    {

        typedef Object* (*instantiateObjectfPtr)(std::shared_ptr<Frame>, configmaps::ConfigMap&);

        class ObjectFactory
        {
        public:
            static ObjectFactory& Instance();
            Object* createObject(std::shared_ptr<Frame> frame, const std::string &type,
                                 configmaps::ConfigMap &config);
            void addObjectType(const std::string& type, instantiateObjectfPtr funcPtr);
            const std::map<const std::string, instantiateObjectfPtr>& getAvailableObjects() const noexcept
            {
                return availableObjects;
            }

        private:
            ObjectFactory();
            ~ObjectFactory();    
            std::map<const std::string, instantiateObjectfPtr> availableObjects;

        };

    }
}
