#include "objects/ObjectFactory.hpp"

#include <memory>

namespace mars
{
    namespace ode_physics
    {

        using namespace ::mars::interfaces;
        using namespace configmaps;

        ObjectFactory& ObjectFactory::Instance()
        {
            static ObjectFactory instance;
            return instance;
        }

        ObjectFactory::ObjectFactory()
        {
        }

        ObjectFactory::~ObjectFactory()
        {
        }

        Object* ObjectFactory::createObject(std::shared_ptr<Frame> frame,
                                            const std::string &type,
                                            ConfigMap &config)
        {

            std::map<const std::string, instantiateObjectfPtr>::iterator it = availableObjects.find(type);
            if(it == availableObjects.end())
            {
                throw std::runtime_error("Could not load unknown Physics Object with name: \"" + config["name"].getString() + "\"" );
            }

            Object *newObject = it->second(frame, config);
            return newObject;
            if(newObject->isObjectCreated())
            {
                return newObject;
            } else
            {
                delete newObject;
                std::cerr << "Failed to create Physics Object with name: \"" + config["name"].getString() + "\"" << std::endl;
                return nullptr;
            }
        }

        void ObjectFactory::addObjectType(const std::string& type, instantiateObjectfPtr funcPtr)
        {
            availableObjects.insert(std::pair<const std::string, instantiateObjectfPtr>(type,funcPtr));
        }

    }
}
