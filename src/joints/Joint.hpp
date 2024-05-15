#pragma once

#include <mars_interfaces/sim/JointInterface.h>
#include <data_broker/ProducerInterface.h>
#include <data_broker/DataPackageMapping.h>
#include <data_broker/DataBrokerInterface.h>


#include "../WorldPhysics.hpp"

namespace mars
{
    namespace ode_physics
    {
        class Joint : public interfaces::JointInterface,
                      public data_broker::ProducerInterface
        {
        public:
            ///the constructor
            Joint(WorldPhysics *world,
                  data_broker::DataBrokerInterface *dataBroker,
                  configmaps::ConfigMap &config);
            ///the destructor
            virtual ~Joint(void);
            ///create Joint getting as argument the JointData which give the
            ///joint informations.
            virtual bool createJoint();
            virtual bool createJoint(dBodyID body1, dBodyID body2);

            virtual void getName(std::string *name) const override;
            virtual interfaces::JointType getType() const override;
            ///get the anchor of the joint
            virtual void getAnchor(utils::Vector* anchor) const override;
            /// set the anchor i.e. the position where the joint is created of the joint
            virtual void setAnchor(const utils::Vector &anchor) override;
            ///set the world informations
            virtual void setAxis(const utils::Vector &axis) override;
            virtual void setAxis2(const utils::Vector &axis) override;
            virtual void getAxis(utils::Vector* axis) const override;
            virtual void getAxis2(utils::Vector* axis) const override;
            virtual void setWorldObject(std::shared_ptr<interfaces::PhysicsInterface>  world) override;
            /// methods to control a joint through a motor
            virtual void setForceLimit(interfaces::sReal max_force) override;
            virtual void setForceLimit2(interfaces::sReal max_force) override;
            virtual void setVelocity(interfaces::sReal velocity) override;
            virtual void setVelocity2(interfaces::sReal velocity) override;
            virtual interfaces::sReal getVelocity(void) const override;
            virtual interfaces::sReal getVelocity2(void) const override;
            virtual void setJointAsMotor(int axis) override;
            virtual void unsetJointAsMotor(int axis) override;
            virtual interfaces::sReal getPosition(void) const override;
            virtual interfaces::sReal getPosition2(void) const override;
            virtual void getForce1(utils::Vector *f) const override;
            virtual void getForce2(utils::Vector *f) const override;
            virtual void getTorque1(utils::Vector *t) const override;
            virtual void getTorque2(utils::Vector *t) const override;
            virtual void setTorque(interfaces::sReal torque) override;
            virtual void setTorque2(interfaces::sReal torque) override;
            virtual void reattacheJoint(void) override;
            virtual void update(void) override;
            virtual void getJointLoad(utils::Vector *t) const override;
            virtual void getAxisTorque(utils::Vector *t) const override;
            virtual void getAxis2Torque(utils::Vector *t) const override;
            virtual void changeStepSize(const interfaces::JointData &jointS) override;
            virtual interfaces::sReal getMotorTorque(void) const override;
            virtual interfaces::sReal getLowStop() const override;
            virtual interfaces::sReal getHighStop() const override;
            virtual interfaces::sReal getLowStop2() const override;
            virtual interfaces::sReal getHighStop2() const override;
            virtual interfaces::sReal getCFM() const override;
            virtual void setLowStop(interfaces::sReal lowStop) override;
            virtual void setHighStop(interfaces::sReal highStop) override;
            virtual void setLowStop2(interfaces::sReal lowStop2) override;
            virtual void setHighStop2(interfaces::sReal highStop2) override;
            virtual void setCFM(interfaces::sReal cfm) override;
            virtual void getDataBrokerNames(std::string& groupName, std::string& dataName) override;

            // --- mars::interfaces::ConfigMapInterface ---
            virtual configmaps::ConfigMap getConfigMap() const override;
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
            virtual void edit(const std::string& configPath, const std::string& value) override;

            bool isJointCreated();

            // ## DataBroker callbacks ##
            virtual void produceData(const data_broker::DataInfo &info,
                                     data_broker::DataPackage *package,
                                     int callbackParam) override;

        protected:
            data_broker::DataBrokerInterface *dataBroker;
            int pushToDataBroker;
            data_broker::DataPackageMapping dbPackageMapping;

            WorldPhysics* theWorld;
            configmaps::ConfigMap config;
            dJointID jointId, ball_motor;
            dJointFeedback feedback;
            dBodyID body1, body2;
            int joint_type;
            dReal cfm, cfm1, cfm2, erp1, erp2;
            dReal lo1, lo2, hi1, hi2;
            dReal damping, spring, jointCFM;
            utils::Vector axis1_torque, axis2_torque, joint_load;
            utils::Vector axis1, axis2, f1, t1, f2, t2, anchor;
            dReal position1, velocity1, position2, velocity2, motor_torque;
            std::string name;

            void calculateCfmErp();
            void addToDataBroker();
            void removeFromDataBroker();
            virtual void updateState() = 0;

        private:
            bool joint_created;
        };

    } // end of namespace ode_physics
} // end of namespace mars
