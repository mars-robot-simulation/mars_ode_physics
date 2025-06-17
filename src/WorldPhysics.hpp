/**
 * \file WorldPhysics.hpp
 * \author Malte Langosz and Team
 * \brief "WorldPhysics" includes the methods to handle the physically world.
 *
 */

#pragma once

//#define _VERIFY_WORLD_
//#define _DEBUG_MASS_

#include <mars_utils/Mutex.h>
#include <mars_utils/Vector.h>
#include <mars_interfaces/sim_common.h>
#include <mars_interfaces/sim/PhysicsInterface.hpp>
#include <data_broker/DataBrokerInterface.h>
#include <configmaps/ConfigMap.hpp>
#include <configmaps/ConfigSchema.hpp>
#include <vector>

#include <ode/ode.h>

namespace mars
{
    namespace ode_physics
    {

        class Object;
        class Frame;
        class ODEJoint;

        /**
         * The struct is used to handle some sensors in the physical
         * implementation with ode. The sensors are not implemented yet.
         */
        // struct robot_geom {
        //     int type;
        //     int pressure1;
        //     int pressure2;
        //     dReal erp;
        //     dReal cfm;
        //     char *texture;
        //     dReal i_length;
        //     dBodyID body;
        //     dReal torque, force;
        // };

        /**
         * Declaration of the physical class, that implements the
         * physics interface.
         */
        class WorldPhysics : public interfaces::PhysicsInterface 
        {
        public:
            explicit WorldPhysics();
            virtual ~WorldPhysics(void) override;
            virtual std::string getLibName(void) override;
            virtual void initTheWorld(void) override;
            virtual void freeTheWorld(void) override;
            virtual void stepTheWorld(void) override;
            virtual void clearPreviousStep(void) override;

            virtual bool existsWorld(void) const override;

            virtual const utils::Vector getCenterOfMass(const std::vector<std::shared_ptr<interfaces::NodeInterface>> &nodes)const override;
            virtual std::shared_ptr<interfaces::DynamicObject> createFrame(data_broker::DataBrokerInterface *dataBroker, configmaps::ConfigMap &config) override;
            virtual std::shared_ptr<interfaces::DynamicObject> getFrame(const std::string& frame) override;
            virtual void removeFrame(const std::string &name) override;
            virtual void createObject(configmaps::ConfigMap &config) override;
            virtual std::shared_ptr<interfaces::JointInterface> createJoint(data_broker::DataBrokerInterface *dataBroker, configmaps::ConfigMap &config) override;
            virtual void destroyJoint(const std::string &jointName) override;
            virtual std::shared_ptr<interfaces::JointInterface> getJoint(std::string name) override;

            virtual void exportWorld() override;

            void addFrame(std::shared_ptr<Frame> frame);
            void registerSchemaValidators();
            // this functions are used by the other physical classes
            dWorldID getWorld(void) const;
            dReal getWorldStep(void);
            std::shared_ptr<Frame> getFrameIntern(const std::string& name);
            const dJointID createContact(dContact &c, dBodyID b1, dBodyID b2);
            void computeContactForces();

            // --- mars::interfaces::ConfigMapInterface ---
            virtual configmaps::ConfigMap getConfigMap() const override;
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
            virtual void edit(const std::string& configPath, const std::string& value) override;

            mutable utils::Mutex iMutex;
            dReal max_angular_speed;
            dReal max_correcting_vel;

            static interfaces::PhysicsError error;
            static int odeLibInitCount;
        private:
            utils::Mutex drawLock;
            dWorldID world;
            dJointGroupID contactgroup;
            dThreadingImplementationID threadingImpl;
            //dThreadingThreadPoolID pool; // required for multi-threading
            bool world_init;
            utils::Vector old_gravity;
            interfaces::sReal old_cfm, old_erp;

            std::vector<dJointFeedback*> contact_feedback_list;
            std::map<std::string, std::weak_ptr<Frame>> frameMap;
            std::map<std::string, std::weak_ptr<interfaces::JointInterface>> jointMap;
            std::vector<Object*> objects;
            void clearObjects();

            bool create_contacts, log_contacts;
            int num_contacts;
            int ray_collision;

            // Step the World auxiliary methods
            void preStepChecks(void);
            std::map<std::string, configmaps::ConfigSchema> objects_schema;
            std::map<std::string, configmaps::ConfigSchema> joints_schema;
        };

    } // end of namespace ode_physics
} // end of namespace mars
