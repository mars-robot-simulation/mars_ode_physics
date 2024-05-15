#include "JointFactory.hpp"
#include <mars_interfaces/utils.h>

#include <memory>

namespace mars
{
    namespace ode_physics
    {

        using namespace ::mars::interfaces;

        JointFactory& JointFactory::Instance()
        {
            static JointFactory instance;
            return instance;
        }

        JointFactory::JointFactory()
        {
        }

        JointFactory::~JointFactory()
        {
        }

        std::shared_ptr<Joint> JointFactory::createJoint(WorldPhysics *world,
                                                         data_broker::DataBrokerInterface *dataBroker,
                                                         configmaps::ConfigMap &config)
        {
            std::map<const std::string, instantiateJointfPtr>::iterator it = availableJoints.find(config["type"].getString());
            if(it == availableJoints.end())
            {
                // todo:
                fprintf(stderr, "Could not load unknown Physics Joint of type: %s\n", config["type"].getString().c_str());
                return nullptr;
            }

            std::shared_ptr<Joint> newJoint(it->second(world, dataBroker, config));
            if(newJoint->isJointCreated())
            {
                return newJoint;
            } else
            {
                std::cerr << "Failed to create Physics Joint with name: \"" + config["name"].getString() + "\"" << std::endl;
                return nullptr;
            }

        }

        void JointFactory::addJointType(const std::string& type, instantiateJointfPtr funcPtr)
        {

            availableJoints.insert(std::pair<const std::string, instantiateJointfPtr>(type, funcPtr));
        }

    }
}
