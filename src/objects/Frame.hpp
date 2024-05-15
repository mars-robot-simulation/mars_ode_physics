 /**
 * \file Frame.hpp
 * \author Malte Langosz and Team
 * \brief "Frame" implements an DynamicObject interface.
 *
 */

#pragma once

#include "../WorldPhysics.hpp"

#include <data_broker/ProducerInterface.h>
#include <data_broker/DataPackageMapping.h>
#include <data_broker/DataBrokerInterface.h>
#include <mars_interfaces/sim/DynamicObject.hpp>
#include <mars_interfaces/Logging.hpp>


namespace mars
{
    namespace ode_physics
    {

        class Frame  : public interfaces::DynamicObject,
                       public data_broker::ProducerInterface
        {
        public:
            Frame(interfaces::PhysicsInterface *world,
                  data_broker::DataBrokerInterface *dataBroker,
                  configmaps::ConfigMap &config);
            ~Frame(void);

            // ## DataBroker callbacks ##
            virtual void produceData(const data_broker::DataInfo &info,
                                     data_broker::DataPackage *package,
                                     int callbackParam) override;

            void addObject(Object*);
            void removeObject(Object* object);

            virtual const std::string& getName() const override;

            virtual void getPosition(utils::Vector *pos) const override;
            virtual void setPosition(const utils::Vector &pos) override;
            virtual void getRotation(utils::Quaternion *q) const override;
            virtual void setRotation(const utils::Quaternion &q) override;
            virtual void addLinkedFrame(std::shared_ptr<DynamicObject> linked) override;
            virtual bool isLinkedFrame(std::shared_ptr<DynamicObject> linked) override;
            virtual std::vector<std::shared_ptr<DynamicObject>> getLinkedFrames(void) override;

            void getLinearVelocity(utils::Vector *vel) const;
            void getAngularVelocity(utils::Vector *vel) const;
            void getForce(utils::Vector *f) const;
            void getTorque(utils::Vector *t) const;

            const utils::Vector rotateAtPoint(const utils::Vector &rotation_point,
                                              const utils::Quaternion &rotation,
                                              bool move_group);
            virtual void setLinearVelocity(const utils::Vector &velocity);
            virtual void setAngularVelocity(const utils::Vector &velocity);
            void setForce(const utils::Vector &f);
            void setTorque(const utils::Vector &t);
            void addForce(const utils::Vector &f, const utils::Vector &p);
            void addForce(const utils::Vector &f);
            void addTorque(const utils::Vector &t);

            virtual void addContact(const interfaces::ContactData& contact) override;
            virtual void getMass(interfaces::sReal *mass, interfaces::sReal *inertia=0) const;

            dBodyID getBody() const;
            dBodyID acquireBody();
            dMass getMass(void) const;
            void getAbsMass(dMass *pMass) const;

            void addObjectMass(Object *object);

            void updateState();

            // --- mars::interfaces::ConfigMapInterface ---
            virtual configmaps::ConfigMap getConfigMap() const override;
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
            virtual void edit(const std::string& configPath, const std::string& value) override;
        protected:
            data_broker::DataBrokerInterface *dataBroker;
            WorldPhysics *theWorld;
            std::string name;
            dBodyID nBody;
            dMass nMass;
            // transformation is handled in absolute coordinates
            utils::Vector position, linearVelocity, angularVelocity, linearAcceleration, angularAcceleration, force, torque;
            utils::Quaternion q;
            std::vector<Object*> objects;

        private:
            // stuff for dataBroker communication
            int pushToDataBroker;
            data_broker::DataPackageMapping dbPackageMapping;
            bool object_created;
            std::vector<std::shared_ptr<DynamicObject>> connectedFrames;
            utils::Vector offsetPos;

            void addToDataBroker();
            void removeFromDataBroker();
        };

    } // end of namespace ode_physics
} // end of namespace mars
