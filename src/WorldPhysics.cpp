/**
 * \file WorldPhysics.cpp
 * \author Malte Langosz and Team
 * \brief "WorldPhysics" includes the methods to handle the physically world.
 *
 * Conditions:
 *           - The state of the private variable world_init is
 *             aquivalent to the initialization state of the world,
 *             space and the contactgroup variables
 *
 * ToDo:
 *               - get and set the standard physical parameters
 *               - get and set the special ODE parameters via
 *                 a generic component through the Simulator class
 *               - handle sensor data
 *
 */

#include "WorldPhysics.hpp"
#include "objects/Object.hpp"
#include "objects/ObjectFactory.hpp"
#include "objects/Frame.hpp"

#include "joints/JointFactory.hpp"
#include "joints/Joint.hpp"

#include <mars_utils/MutexLocker.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <mars_interfaces/sim/JointInterface.h>
#include <mars_interfaces/Logging.hpp>
#include <mars_utils/misc.h>

#define EPSILON 1e-10


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;

        PhysicsError WorldPhysics::error = PHYSICS_NO_ERROR;
        bool WorldPhysics::odeLibIsInit = false;

        void myMessageFunction(int errnum, const char *msg, va_list ap)
        {
            CPP_UNUSED(errnum);
            LOG_INFO(msg, ap);
        }

        void myDebugFunction(int errnum, const char *msg, va_list ap)
        {
            CPP_UNUSED(errnum);
            LOG_DEBUG(msg, ap);
            WorldPhysics::error = PHYSICS_DEBUG;
        }

        void myErrorFunction(int errnum, const char *msg, va_list ap)
        {
            CPP_UNUSED(errnum);
            LOG_ERROR(msg, ap);
            WorldPhysics::error = PHYSICS_ERROR;
        }

        /**
         *  \brief The constructor for the physical world.
         *
         *  pre:
         *      - none
         *
         *  post:
         *      - all private variables should be initialized correct
         *        should correct be spezified?
         *      - world, space, contactgroup and world_init to false (0)
         */
        WorldPhysics::WorldPhysics()
        {
            fast_step = 0;
            world_cfm = 1e-10;
            world_erp = 0.1;
            world_gravity = Vector(0.0, 0.0, -9.81);
            ground_friction = 20;
            ground_cfm = 0.00000001;
            ground_erp = 0.1;
            world = 0;
            contactgroup = 0;
            world_init = 0;
            num_contacts = 0;
            create_contacts = 1;
            log_contacts = 0;
            max_angular_speed = 10.0; // I guess this is rad/s
            max_correcting_vel = 5.0;

            // the step size in seconds
            step_size = 0.01;
            // dInitODE is relevant for using trimesh objects as correct as
            // possible in the ode implementation
            MutexLocker locker(&iMutex);
            if (!odeLibIsInit)
            {
                fprintf(stderr, "............ call dInitODE2\n");
#ifdef ODE11
                // for ode-0.11
                dInitODE2(0);
#else
                dInitODE();
#endif
                dSetErrorHandler(myErrorFunction);
                dSetDebugHandler(myDebugFunction);
                dSetMessageHandler(myMessageFunction);
                odeLibIsInit = true;
            }

            dAllocateODEDataForThread(dAllocateMaskAll);
            registerSchemaValidators();
        }
        /**
         * \brief Close ODE environment
         *
         * pre:
         *     - none
         *
         * post:
         *     - everthing that was created should be destroyed
         *
         */
        WorldPhysics::~WorldPhysics(void)
        {
            // free the ode objects
            freeTheWorld();
            // and close the ODE ...
            MutexLocker locker(&iMutex);
            dCloseODE();
        }

        std::string WorldPhysics::getLibName(void)
        {
            return "mars_ode_physics";
        }

        /**
         *  \brief This function initializes the ode world.
         *
         * pre:
         *     - world_init = false
         *
         * post:
         *     - world, space and contact group should be created
         *     - the ODE world parameters should be set
         *     - at the end world_init have to become true
         */
        void WorldPhysics::initTheWorld(void)
        {
            MutexLocker locker(&iMutex);

            // if world_init = true debug something
            if (!world_init)
            {
                // LOG_DEBUG("init physics world");
                world = dWorldCreate();

                dThreadingImplementationID threadingImpl = dThreadingAllocateSelfThreadedImplementation();
                const dThreadingFunctionsInfo *functions = dThreadingImplementationGetFunctions(threadingImpl);
                dWorldSetStepThreadingImplementation(world, functions, threadingImpl);

                contactgroup = dJointGroupCreate(0);

                old_gravity = world_gravity;
                old_cfm = world_cfm;
                old_erp = world_erp;

                dWorldSetGravity(world, world_gravity.x(), world_gravity.y(), world_gravity.z());
                dWorldSetCFM(world, (dReal)world_cfm);
                dWorldSetERP(world, (dReal)world_erp);
                dWorldSetMaxAngularSpeed(world, max_angular_speed);
                dWorldSetContactMaxCorrectingVel(world, max_correcting_vel);

                dWorldSetAutoDisableFlag(world, 0);
                world_init = 1;
            }
        }

        /**
         * \brief This functions destroys the ode world.
         *
         * pre:
         *     - world_init = true
         *
         * post:
         *     - world, space and contactgroup have to be destroyed here
         *     - afte that, world_init have to become false
         */
        void WorldPhysics::freeTheWorld(void)
        {
            MutexLocker locker(&iMutex);
            if (world_init)
            {
                // LOG_DEBUG("free physics world");
                dJointGroupDestroy(contactgroup);
                dWorldDestroy(world);
                world_init = 0;
            }
            // else debug something
        }

        /**
         * \brief Returns if a world exists.
         *
         * pre:
         *     - none
         *
         * post:
         *     - return state of world_init
         */
        bool WorldPhysics::existsWorld(void) const
        {
            return world_init;
        }

        /**
         * \brief Register schema validators for objects and joints of this world
         * \note config will be validated by the corresponding schema before the creation of any Object/Joint by WorldPhysics::createObject/Joint
         */
        void WorldPhysics::registerSchemaValidators()
        {
            // Register schema validators for objects of this world
            for (const auto &[obj_name, _] : ObjectFactory::Instance().getAvailableObjects())
            {
                std::string obj_schema_path = mars::utils::pathJoin(SCHEMA_PATH, "objects/" + obj_name + "_schema.yaml");
                if (!mars::utils::pathExists(obj_schema_path))
                {
                    LOG_ERROR("Missing schema file " + obj_schema_path + " for " + obj_name);
                    continue;
                }
                configmaps::ConfigSchema schema(configmaps::ConfigMap::fromYamlFile(obj_schema_path));
                objects_schema[obj_name] = schema;
            }
            // Register schema validators for joints of this world
            for (const auto &[joint_name, _] : JointFactory::Instance().getAvailableJoints())
            {
                std::string joint_schema_path = mars::utils::pathJoin(SCHEMA_PATH, "joints/" + joint_name + "_schema.yaml");
                if (!mars::utils::pathExists(joint_schema_path))
                {
                    LOG_ERROR("Missing schema file " + joint_schema_path + " for " + joint_name);
                    continue;
                }
                configmaps::ConfigSchema schema(configmaps::ConfigMap::fromYamlFile(joint_schema_path));
                joints_schema[joint_name] = schema;
            }
        }
        /**
         *
         * \brief Auxiliary method of of step the world. Checks required before
         * computing the collision and the contact forces are done here.
         *
         * The checks consist on: updating gravity, cfm and erp for coherence.
         *
         */
        void WorldPhysics::preStepChecks(void)
        {
            if (old_gravity != world_gravity)
            {
                old_gravity = world_gravity;
                dWorldSetGravity(world, world_gravity.x(),
                                 world_gravity.y(), world_gravity.z());
            }

            if (old_cfm != world_cfm)
            {
                old_cfm = world_cfm;
                dWorldSetCFM(world, (dReal)world_cfm);
            }

            if (old_erp != world_erp)
            {
                old_erp = world_erp;
                dWorldSetERP(world, (dReal)world_erp);
            }
        }

        /**
         *
         * \brief Auxiliary method of step the world.
         * Clears the contact and contact feedback information from previous step.
         *
         */
        void WorldPhysics::clearPreviousStep(void)
        {
            // Clear contacts
            dJointGroupEmpty(contactgroup);
        }

        /**
         * \brief This function handles the calculation of a step in the world.
         *
         * pre:
         *     - world_init = true
         *     - step_size > 0
         *
         * post:
         *     - handled the collisions
         *     - step the world for step_size seconds
         *     - the contactgroup should be empty
         */
        void WorldPhysics::stepTheWorld(void)
        {
            MutexLocker locker(&iMutex);
            std::vector<dJointFeedback *>::iterator iter;

            // if world_init = false or step_size <= 0 debug something
            if (world_init && step_size > 0)
            {
                preStepChecks();
                /// first check for collisions
                num_contacts = log_contacts = 0;

                /// then calculate the next state for a time of step_size seconds
                try
                {
                    if (fast_step)
                        dWorldQuickStep(world, step_size);
                    else
                        dWorldStep(world, step_size);

                    for (auto it = jointMap.begin(); it != jointMap.end();)
                    {
                        if (auto joint = it->second.lock())
                        {
                            joint->update();
                            ++it;
                        }
                        else
                        {
                            // The joint was deleted => remove it from mapping
                            jointMap.erase(it++);
                        }
                    }
                }
                catch (int id)
                {
                    // TODO Check that you really need this before doing the patch
                    if (id == 3)
                        LOG_ERROR("Problem normalizing a vector");
                    if (id == 4)
                        LOG_ERROR("Problem normalizing a quaternion");
                }
                catch (...)
                {
                    // control->sim->handleError(PHYSICS_UNKNOWN);
                }
                if (WorldPhysics::error)
                {
                    // control->sim->handleError(WorldPhysics::error);
                    WorldPhysics::error = PHYSICS_NO_ERROR;
                }
            }
        }

        /**
         * \brief Returns the ode ID of the world object.
         *
         * pre:
         *     - none
         *
         * post:
         *     - world ID returned
         */
        dWorldID WorldPhysics::getWorld(void) const
        {
            return world;
        }

        void WorldPhysics::exportWorld()
        {
            std::cout << "EXPORT WORLD" << std::endl;
            FILE* fp = stdout;
            dWorldExportDIF(getWorld(), fp, "");
        }

        /**
         * \brief Returns the stepsize for calculating a world step
         *
         * pre:
         *     - none
         *
         * post:
         *     - the step_size value should be returned
         */
        dReal WorldPhysics::getWorldStep(void)
        {
            return step_size;
        }

        void WorldPhysics::createContact(dContact &c, dBodyID b1, dBodyID b2)
        {
            // ensure that bodies are not connected by joint
            if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
                return;
            dJointID joint = dJointCreateContact(world, contactgroup, &c);
            dJointAttach(joint, b1, b2);
        }

        const Vector WorldPhysics::getCenterOfMass(const std::vector<std::shared_ptr<NodeInterface>> &nodes) const
        {
            // todo:
            // MutexLocker locker(&iMutex);
            // Vector center;
            // std::vector<std::shared_ptr<NodeInterface>>::const_iterator iter;
            // dMass sumMass;
            // dMass tMass;

            // dMassSetZero(&sumMass);
            // for(iter = nodes.begin(); iter != nodes.end(); iter++) {
            //     (std::static_pointer_cast<ODEObject>(*iter))->getAbsMass(&tMass);
            //     dMassAdd(&sumMass, &tMass);
            // }
            // center.x() = sumMass.c[0];
            // center.y() = sumMass.c[1];
            // center.z() = sumMass.c[2];
            // return center;
            return Vector();
        }

        std::shared_ptr<DynamicObject> WorldPhysics::createFrame(data_broker::DataBrokerInterface *dataBroker, configmaps::ConfigMap &config)
        {
            std::shared_ptr<Frame> frame(new Frame(this, dataBroker, config));
            addFrame(frame);
            return frame;
        }

        void WorldPhysics::addFrame(std::shared_ptr<Frame> frame)
        {
            const std::string &name = frame->getName();
            if (frameMap.find(name) == frameMap.end())
            {
                frameMap[name] = frame;
            }
            else
            {
                // todo: add proper error handling
            }
        }

        void WorldPhysics::removeFrame(const std::string &name)
        {
            if (frameMap.find(name) != frameMap.end())
            {
                frameMap.erase(name);
            }
            else
            {
                // todo: add proper error handling
            }
        }

        std::shared_ptr<Frame> WorldPhysics::getFrameIntern(const std::string &name)
        {
            auto it = frameMap.find(name);
            if (it != frameMap.end())
            {
                return it->second;
            }
            else
            {
                // todo: add proper error handling
            }
            return nullptr;
        }

        std::shared_ptr<DynamicObject> WorldPhysics::getFrame(const std::string &name)
        {
            std::shared_ptr<Frame> frame = getFrameIntern(name);
            if (frame)
            {
                return frame;
            }
            else
            {
                // todo: add proper error handling
            }
            return nullptr;
        }

        void WorldPhysics::createObject(configmaps::ConfigMap &config)
        {
            std::string type = config["type"];
            if (!objects_schema.count(type))
            {
                const std::string errmsg = "WorldPhysics::createObject: could not create object " + type + ": missing schema file.";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            if (!objects_schema[type].validate(config))
            {
                const std::string errmsg = "WorldPhysics::createObject: could not create object " + type + ": invalid config.";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            std::shared_ptr<Frame> frame = getFrameIntern(config["parentFrame"]);
            if(frame == nullptr)
            {
                const std::string errmsg = "WorldPhysics::createObject: could not create object " + type + ": missing frame: " + config["parentFrame"].toString() + ".";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            // todo: if frame not found return error
            ObjectFactory::Instance().createObject(frame, type, config);
        }

        std::shared_ptr<JointInterface> WorldPhysics::createJoint(data_broker::DataBrokerInterface *dataBroker, configmaps::ConfigMap &config)
        {
            std::string name = config["name"];
            std::string type = config["type"];
            if (!joints_schema.count(type))
            {
                const std::string errmsg = "WorldPhysics::createJoint: could not create joint " + type + ": missing schema file.";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            if (!joints_schema[type].validate(config))
            {
                const std::string errmsg = "WorldPhysics::createJoint: could not create joint " + type + ": invalid config.";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            std::shared_ptr<Joint> newJoint = JointFactory::Instance().createJoint(this, dataBroker, config);
            jointMap[name] = newJoint;
            return newJoint;
            // newJoint->setVelocity(0);
            // newJoint->setForceLimit(100.0);
        }

        std::shared_ptr<JointInterface> WorldPhysics::getJoint(std::string name)
        {
            auto it = jointMap.find(name);
            if (it != jointMap.end())
            {
                return it->second.lock();
            }
            return nullptr;
        }

    } // end of namespace ode_physics
} // end of namespace mars
