/**
 * \file WorldPhysicsLoader.cpp
 * \author Malte Langosz
 * \brief "WorldPhysicsLoader" factory class to generate new physics instances.
 *
 */

#include "WorldPhysicsLoader.hpp"
#include "objects/ObjectFactory.hpp"
#include "objects/Box.hpp"
#include "objects/Inertial.hpp"
#include "joints/JointFactory.hpp"
#include "joints/HingeJoint.hpp"
#include "joints/FixedJoint.hpp"
#include "joints/SliderJoint.hpp"

namespace mars
{
    namespace ode_physics
    {

        WorldPhysicsLoader::WorldPhysicsLoader(lib_manager::LibManager *theManager) : lib_manager::LibInterface(theManager)
        {
            ObjectFactory::Instance().addObjectType("box", &Box::instantiate);
            ObjectFactory::Instance().addObjectType("inertial", &Inertial::instantiate);
            JointFactory::Instance().addJointType("hinge", &HingeJoint::instantiate);
            JointFactory::Instance().addJointType("fixed", &FixedJoint::instantiate);
            JointFactory::Instance().addJointType("slider", &SliderJoint::instantiate);
            JointFactory::Instance().addJointType("prismatic", &SliderJoint::instantiate);
        }

        WorldPhysicsLoader::~WorldPhysicsLoader(void) {

        }

        std::shared_ptr<interfaces::PhysicsInterface> WorldPhysicsLoader::createWorldInstance()
        {
            std::shared_ptr<WorldPhysics> worldPhysics = std::make_shared<WorldPhysics>();
            return std::static_pointer_cast<interfaces::PhysicsInterface>(worldPhysics);
        }

    } // end of namespace ode_physics
} // end of namespace mars

DESTROY_LIB(mars::ode_physics::WorldPhysicsLoader)
CREATE_LIB(mars::ode_physics::WorldPhysicsLoader)
