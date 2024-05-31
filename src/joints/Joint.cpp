/*
 *  \file Joint.cpp
 *  \autor Malte Langosz
 *  \brief "Joint" declares the physics for the joint using ode
 *
 */

#include <mars_utils/MutexLocker.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>       // matchPatttern
#include <mars_interfaces/utils.h>

#include "Joint.hpp"
#include "../objects/Object.hpp"

#include <cstdio>
#include <iostream>

namespace mars
{
    namespace ode_physics
    {
        using namespace utils;
        using namespace interfaces;

        /**
         * \brief the constructor of the Joint physics
         *   initialize the attributes of the object
         */
        Joint::Joint(WorldPhysics *world,
                     data_broker::DataBrokerInterface *dataBroker,
                     configmaps::ConfigMap &config) : 
                     theWorld(world), dataBroker(dataBroker), config(config)
        {
            jointId = ball_motor = 0;
            jointCFM = 0.0;
            cfm = cfm1 = cfm2 = erp1 = erp2 = 0;
            lo1 = lo2 = hi1 = hi2 = 0;
            damping = 0;
            spring = 0;
            body1 = 0;
            body2 = 0;
            joint_created = false;
            name << config["name"];

            pushToDataBroker = 2; // push all data TODO: use enum
            if(config.hasKey("pushToDataBroker"))
            {
                pushToDataBroker = config["pushToDataBroker"];
            } else if(config.hasKey("noDataPackage") && (bool)config["noDataPackage"] == true)
            {
                pushToDataBroker = 0;
            } else if(config.hasKey("reducedDataPackage") && (bool)config["reducedDataPackage"] == true)
            {
                pushToDataBroker = 1;
            }

            if(pushToDataBroker > 0)
            {
                dbPackageMapping.clear();
                dbPackageMapping.add("axis1/angle", &position1);
                dbPackageMapping.add("axis1/speed", &velocity1);
                dbPackageMapping.add("axis1/torque/x", &axis1_torque.x());
                dbPackageMapping.add("axis1/torque/y", &axis1_torque.y());
                dbPackageMapping.add("axis1/torque/z", &axis1_torque.z());
                dbPackageMapping.add("jointLoad/x", &joint_load.x());
                dbPackageMapping.add("jointLoad/y", &joint_load.y());
                dbPackageMapping.add("jointLoad/z", &joint_load.z());
                dbPackageMapping.add("motorTorque", &motor_torque);
            }
            if(pushToDataBroker > 1)
            {
                dbPackageMapping.add("axis1/x", &axis1.x());
                dbPackageMapping.add("axis1/y", &axis1.y());
                dbPackageMapping.add("axis1/z", &axis1.z());
                dbPackageMapping.add("force1/x", &f1.x());
                dbPackageMapping.add("force1/y", &f1.y());
                dbPackageMapping.add("force1/z", &f1.z());
                dbPackageMapping.add("torque1/x", &t1.x());
                dbPackageMapping.add("torque1/y", &t1.y());
                dbPackageMapping.add("torque1/z", &t1.z());

                dbPackageMapping.add("axis2/x", &axis2.x());
                dbPackageMapping.add("axis2/y", &axis2.y());
                dbPackageMapping.add("axis2/z", &axis2.z());
                dbPackageMapping.add("axis2/angle", &position2);
                dbPackageMapping.add("axis2/speed", &velocity2);
                dbPackageMapping.add("axis2/velocity", &velocity2);
                dbPackageMapping.add("axis2/torque/x", &axis2_torque.x());
                dbPackageMapping.add("axis2/torque/y", &axis2_torque.y());
                dbPackageMapping.add("axis2/torque/z", &axis2_torque.z());

                dbPackageMapping.add("force2/x", &f2.x());
                dbPackageMapping.add("force2/y", &f2.y());
                dbPackageMapping.add("force2/z", &f2.z());
                dbPackageMapping.add("torque2/x", &t2.x());
                dbPackageMapping.add("torque2/y", &t2.y());
                dbPackageMapping.add("torque2/z", &t2.z());

                dbPackageMapping.add("anchor/x", &anchor.x());
                dbPackageMapping.add("anchor/y", &anchor.y());
                dbPackageMapping.add("anchor/z", &anchor.z());
            }
            if(pushToDataBroker > 0)
            {
                addToDataBroker();
            }
        }

        /**
         * \brief Destroys the joint in the physics world.
         *
         * pre:
         *     - theWorld is the correct world object
         *
         * post:
         *     - all physical representation of the joint should be cleared
         */
        Joint::~Joint(void)
        {
            const MutexLocker locker{&(theWorld->iMutex)};
            if (jointId)
            {
                dJointDestroy(jointId);
            }
            removeFromDataBroker();
        }

        void Joint::getName(std::string *name) const
        {
            *name = this->name;
        }

        interfaces::JointType Joint::getType() const
        {
            return static_cast<interfaces::JointType>(joint_type);
        }

        void Joint::calculateCfmErp()
        {
            // how to set erp and cfm
            // ERP = h kp / (h kp + kd)
            // CFM = 1 / (h kp + kd)
            damping = -1.0;
            spring = -1.0;
            if(config.hasKey("springDamping"))
            {
                damping = config["springDamping"];
            }
            if(config.hasKey("damping_const_constraint_axis1"))
            {
                damping = config["damping_const_constraint_axis1"];
            }

            if(config.hasKey("springStiffness"))
            {
                spring = config["springStiffness"];
            }
            if(config.hasKey("spring_const_constraint_axis1"))
            {
                spring = config["spring_const_constraint_axis1"];
            }
            dReal h = theWorld->getWorldStep();
            cfm = damping;
            erp1 = h*spring + damping;
            cfm1 = h*spring + damping;
/*
            erp2 = h*(dReal)jointS->spring_const_constraint_axis2
                +(dReal)jointS->damping_const_constraint_axis2;
            cfm2 = h*(dReal)jointS->spring_const_constraint_axis2
                +(dReal)jointS->damping_const_constraint_axis2;
*/
            lo1 = -dInfinity;
            hi1 = dInfinity;
            if(config.hasKey("minPosition"))
            {
                hi1 = -1*(double)config["minPosition"];
            }
            if(config.hasKey("maxPosition"))
            {
                lo1 = -1*(double)config["maxPosition"];
            }
            // TODO: currently the smurf modelling does not fit to what is required in ode so we set ignore lo and hi stops in general and use them only to apply spring and damping parameters while both are set to zero:
            lo1 = 0.0;
            hi1 = 0.0;
/*
            hi2 = jointS->highStopAxis2;
*/
            // we don't want to run in the trap where kp and kd are set to zero
            cfm = (cfm>0)?1/cfm:0.000000000001;
            erp1 = (erp1>0)?h*(dReal)spring/erp1:0;
            cfm1 = (cfm1>0)?1/cfm1:0.00000000001;
/*
            erp2 = (erp2>0)?h*(dReal)jointS->spring_const_constraint_axis2/erp2:0;
            cfm2 = (cfm2>0)?1/cfm2:0.000000000001;
*/
        }

        bool Joint::isJointCreated()
        {
            return joint_created;
        }

        /**
         * \brief create the joint with the informations giving from jointS
         *
         */
        bool Joint::createJoint()
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(theWorld && theWorld->existsWorld())
            {
                //get the bodies from the interfaces nodes
                //here we have to make some verifications
                // TODO: we should store the joint in the frames
                fprintf(stderr, "joint type: %s\n", config["type"].getString().c_str());
                fprintf(stderr, "connect: %s --- %s\n", config["parent_link_name"].getString().c_str(), config["child_link_name"].getString().c_str());
                std::shared_ptr<Frame> n1 = theWorld->getFrameIntern(config["parent_link_name"]);
                if (n1 == nullptr)
                {
                    LOG_ERROR("Can not create a new joint, since no parent link with the name " + config["parent_link_name"].toString() + " was found.");
                    return false;
                }
                std::shared_ptr<Frame> n2 = theWorld->getFrameIntern(config["child_link_name"]);
                if (n2 == nullptr)
                {
                    LOG_ERROR("Can not create a new joint, since no child link with the name " + config["child_link_name"].toString() + " was found.");
                    return false;
                }

                dBodyID b1 = 0, b2 = 0;

                calculateCfmErp();
                if(config.hasKey("jointCFM"))
                {
                    jointCFM = config["jointCFM"];
                }

                // TODO: fix joint type from string to int
                joint_type = getJointType(config["type"].getString());

                if(n1) b1 = n1->acquireBody();
                if(n2) b2 = n2->acquireBody();
                body1 = b1;
                body2 = b2;

                bool ret = createJoint(body1, body2);
                if(!ret)
                {
                    // Error creating the joint
                    return false;
                }

                if(n1 && n2)
                {
                    n1->addLinkedFrame(n2);
                    n2->addLinkedFrame(n1);
                }

                // we have created a joint. To get the information
                // of the forces the joint attached to the bodies
                // we need to set a feedback pointer for the joint (ode stuff)
                dJointSetFeedback(jointId, &feedback);
                joint_created = true;
                return true;
            }
            return false;
        }

        bool Joint::createJoint(dBodyID body1, dBodyID body2)
        {
            std::cout << "Joint: using default createJoint func. Did you forget to override it?." << std::endl;
            LOG_WARN("Object: using default createJoint func. Did you forget to override it?.");
            return 0;
        }

        ///get the anchor of the joint
        void Joint::getAnchor(Vector* anchor) const
        {
            dReal pos[4] = {0,0,0,0};
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type) 
            {
            case  JOINT_TYPE_HINGE:
                dJointGetHingeAnchor(jointId, pos);
                break;
            case JOINT_TYPE_HINGE2:
                dJointGetHinge2Anchor(jointId, pos);
                break;
            case JOINT_TYPE_SLIDER:
                // the slider joint has no ancher point
                break;
            case JOINT_TYPE_BALL:
                dJointGetBallAnchor(jointId, pos);
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointGetUniversalAnchor(jointId, pos);
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
            anchor->x() = pos[0];
            anchor->y() = pos[1];
            anchor->z() = pos[2];
        }

        // the next force and velocity methods are only in a beta state
        void Joint::setForceLimit(sReal max_force)
        {
            MutexLocker locker(&(theWorld->iMutex));
            //fprintf(stderr, "setForceLimit: %g\n", max_force);
            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointSetHingeParam(jointId, dParamFMax, (dReal)max_force);
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamFMax, (dReal)max_force);
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderParam(jointId, dParamFMax, (dReal)max_force);
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalParam(jointId, dParamFMax, (dReal)max_force);
                break;
            }
        }

        void Joint::setForceLimit2(sReal max_force)
        {
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamFMax2, (dReal)max_force);
                break;
            case JOINT_TYPE_SLIDER:
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalParam(jointId, dParamFMax2, (dReal)max_force);
                break;
            }
        }

        void Joint::setVelocity(sReal velocity)
        {
            MutexLocker locker(&(theWorld->iMutex));
            //fprintf(stderr, "setVelocity: %g\n", -velocity);
            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointSetHingeParam(jointId, dParamVel, (dReal)-velocity);
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamVel, (dReal)velocity);
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderParam(jointId, dParamVel, (dReal)-velocity);
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalParam(jointId, dParamVel, (dReal)velocity);
                break;
            }
        }

        void Joint::setVelocity2(sReal velocity)
        {
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamVel2, (dReal)velocity);
                break;
            case JOINT_TYPE_SLIDER:
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalParam(jointId, dParamVel2, (dReal)velocity);
                break;
            }
        }

        sReal Joint::getPosition(void) const
        {
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                return (sReal)dJointGetHingeAngle(jointId)*-1;
                break;
            case JOINT_TYPE_HINGE2:
                return (sReal)dJointGetHinge2Angle1(jointId);
                break;
            case JOINT_TYPE_SLIDER:
                return (sReal)dJointGetSliderPosition(jointId)*-1;
                break;
            case JOINT_TYPE_UNIVERSAL:
                return (sReal)dJointGetUniversalAngle1(jointId);
                break;
            }
            return 0;
        }

        sReal Joint::getPosition2(void) const
        {
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case JOINT_TYPE_UNIVERSAL:
                return (sReal)dJointGetUniversalAngle2(jointId);
                break;
            }
            return 0;
        }

        /// set the anchor i.e. the position where the joint is created of the joint
        void Joint::setAnchor(const Vector &anchor)
        {
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointSetHingeAnchor(jointId, anchor.x(), anchor.y(), anchor.z());
                //std::cout << " " << anchor.z();
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Anchor(jointId, anchor.x(), anchor.y(), anchor.z());
                break;
            case JOINT_TYPE_SLIDER:
                // the slider joint has no ancher point
                break;
            case JOINT_TYPE_BALL:
                dJointSetBallAnchor(jointId, anchor.x(), anchor.y(), anchor.z());
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalAnchor(jointId, anchor.x(), anchor.y(), anchor.z());
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
        }

        /**
         * \brief Set the Axis of the Joint to a new position
         *
         * pre:
         *
         * post:
         */
        void Joint::setAxis(const Vector &axis)
        {
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointSetHingeAxis(jointId, axis.x(), axis.y(), axis.z());
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Axis1(jointId, axis.x(), axis.y(), axis.z());
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderAxis(jointId, axis.x(), axis.y(), axis.z());
                break;
            case JOINT_TYPE_BALL:
                // the ball joint has no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalAxis1(jointId, axis.x(), axis.y(), axis.z());
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
        }

        /**
         * \brief Set the Axis2 of the Joint
         *
         * pre:
         *
         * post:
         */
        void Joint::setAxis2(const Vector &axis)
        {
            MutexLocker locker(&(theWorld->iMutex));
            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                // the hinge joint has only one axis
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Axis2(jointId, axis.x(), axis.y(), axis.z());
                break;
            case JOINT_TYPE_SLIDER:
                // the slider joint has only one axis
                break;
            case JOINT_TYPE_BALL:
                // the ball joint has no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointSetUniversalAxis2(jointId, axis.x(), axis.y(), axis.z());
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
        }

        /**
         * \brief Gets the actual axis of a joint
         *
         * pre:
         *     - the joint should be created and should have a axis vector
         *
         * post:
         *     - the given axis struct should be filled with correct values
         */
        void Joint::getAxis(Vector* axis) const
        {
            dReal pos[4] = {0,0,0,0};
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointGetHingeAxis(jointId, pos);
                break;
            case JOINT_TYPE_HINGE2:
                dJointGetHinge2Axis1(jointId, pos);
                break;
            case JOINT_TYPE_SLIDER:
                dJointGetSliderAxis(jointId, pos);
                break;
            case JOINT_TYPE_BALL:
                // the ball joint has no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointGetUniversalAxis1(jointId, pos);
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
            axis->x() = (sReal)pos[0];
            axis->y() = (sReal)pos[1];
            axis->z() = (sReal)pos[2];
        }

        /**
         * \brief Gets the actual second axis of a joint
         *
         * pre:
         *     - the joint should be created and should have second axis vector
         *
         * post:
         *     - the given axis struct should be filled with correct values
         */
        void Joint::getAxis2(Vector* axis) const
        {
            dReal pos[4] = {0,0,0,0};
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                // the hinge joint has only one axis
                break;
            case JOINT_TYPE_HINGE2:
                dJointGetHinge2Axis2(jointId, pos);
                break;
            case JOINT_TYPE_SLIDER:
                // the slider joint has only one axis
                break;
            case JOINT_TYPE_BALL:
                // the ball joint has no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointGetUniversalAxis2(jointId, pos);
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
            axis->x() = (sReal)pos[0];
            axis->y() = (sReal)pos[1];
            axis->z() = (sReal)pos[2];
        }

        ///set the world informations
        void Joint::setWorldObject(std::shared_ptr<PhysicsInterface>  world)
        {
            //TODO: theWorld = std::static_pointer_cast<WorldPhysics>(world);
        }

        void Joint::setJointAsMotor(int axis)
        {
            MutexLocker locker(&(theWorld->iMutex));
            switch(joint_type)
            {
                // TODO: need to handle the distinction whether to set or not to
                //       set the hi and low stop differently
            case  JOINT_TYPE_HINGE:
            {
                dJointSetHingeParam(jointId, dParamLoStop, -dInfinity);
                dJointSetHingeParam(jointId, dParamHiStop, dInfinity);
            }
            break;
            case JOINT_TYPE_HINGE2:
                if(!lo1 && !hi1 && axis == 1)
                {
                    dJointSetHinge2Param(jointId, dParamLoStop, -dInfinity);
                    dJointSetHinge2Param(jointId, dParamHiStop, dInfinity);
                }
                if(!lo2 && !hi2 && axis == 2)
                {
                    dJointSetHinge2Param(jointId, dParamLoStop2, -dInfinity);
                    dJointSetHinge2Param(jointId, dParamHiStop2, dInfinity);
                }
                break;
            case JOINT_TYPE_SLIDER:
                if(!lo1 && !hi1)
                {
                    dJointSetSliderParam(jointId, dParamLoStop, -dInfinity);
                    dJointSetSliderParam(jointId, dParamHiStop, dInfinity);
                }
                break;
            case JOINT_TYPE_UNIVERSAL:
                /*
                  if(!lo1 && !hi1 && axis == 1) {
                  dJointSetUniversalParam(jointId, dParamLoStop, -dInfinity);
                  dJointSetUniversalParam(jointId, dParamHiStop, dInfinity);
                  }
                  if(!lo2 && !hi2 && axis == 2) {
                  dJointSetUniversalParam(jointId, dParamLoStop2, -dInfinity);
                  dJointSetUniversalParam(jointId, dParamHiStop2, dInfinity);
                  }
                */
                break;
            }
        }

        void Joint::unsetJointAsMotor(int axis)
        {
            MutexLocker locker(&(theWorld->iMutex));
            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointSetHingeParam(jointId, dParamLoStop, lo1);
                dJointSetHingeParam(jointId, dParamHiStop, hi1);
                break;
            case JOINT_TYPE_HINGE2:
                if(axis == 1)
                {
                    dJointSetHinge2Param(jointId, dParamLoStop, lo1);
                    dJointSetHinge2Param(jointId, dParamHiStop, hi1);
                }
                else if(axis == 2)
                {
                    dJointSetHinge2Param(jointId, dParamLoStop2, lo2);
                    dJointSetHinge2Param(jointId, dParamHiStop2, hi2);
                }
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderParam(jointId, dParamLoStop, lo1);
                dJointSetSliderParam(jointId, dParamHiStop, hi1);
                break;
            case JOINT_TYPE_UNIVERSAL:
                if(axis == 1)
                {
                    dJointSetUniversalParam(jointId, dParamLoStop, lo1);
                    dJointSetUniversalParam(jointId, dParamHiStop, hi1);
                }
                else if(axis == 2)
                {
                    dJointSetUniversalParam(jointId, dParamLoStop2, lo2);
                    dJointSetUniversalParam(jointId, dParamHiStop2, hi2);
                }
                break;
            }
        }

        /**
         * \brief Gets the force of the joint which applies to the first body
         *
         * pre:
         *     - the joint should be created
         *
         * post:
         *
         */
        void Joint::getForce1(Vector *f) const
        {
            f->x() = (sReal)feedback.f1[0];
            f->y() = (sReal)feedback.f1[1];
            f->z() = (sReal)feedback.f1[2];
        }

        /**
         * \brief Gets the force of the joint which applies to the second body
         *
         * pre:
         *     - the joint should be created
         *
         * post:
         *
         */
        void Joint::getForce2(Vector *f) const
        {
            f->x() = (sReal)feedback.f2[0];
            f->y() = (sReal)feedback.f2[1];
            f->z() = (sReal)feedback.f2[2];
        }

        /**
         * \brief Gets the torque of the joint which applies to the first body
         *
         * pre:
         *     - the joint should be created
         *
         * post:
         *
         */
        void Joint::getTorque1(Vector *t) const
        {
            t->x() = (sReal)feedback.t1[0];
            t->y() = (sReal)feedback.t1[1];
            t->z() = (sReal)feedback.t1[2];
        }

        /**
         * \brief Gets the torque the joint which applies to the second body
         *
         * pre:
         *     - the joint should be created
         *
         * post:
         *
         */
        void Joint::getTorque2(Vector *t) const
        {
            t->x() = (sReal)feedback.t2[0];
            t->y() = (sReal)feedback.t2[1];
            t->z() = (sReal)feedback.t2[2];
        }

        /**
         * \brief reset the anchor to the actual position.  If a Node is moved or
         * rotated by the editor, this function resets the constrains the joint
         * applies to the Nodes that are connected.
         *
         * pre:
         *     - the joint should be created
         *
         * post:
         *
         */
        void Joint::reattacheJoint(void)
        {
            dReal pos[4] = {0,0,0,0};
            MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointGetHingeAnchor(jointId, pos);
                dJointSetHingeAnchor(jointId, pos[0], pos[1], pos[2]);
                break;
            case JOINT_TYPE_HINGE2:
                dJointGetHinge2Anchor(jointId, pos);
                dJointSetHinge2Anchor(jointId, pos[0], pos[1], pos[2]);
                break;
            case JOINT_TYPE_SLIDER:
                // the slider joint has no anchor point
                break;
            case JOINT_TYPE_BALL:
                dJointGetBallAnchor(jointId, pos);
                dJointSetBallAnchor(jointId, pos[0], pos[1], pos[2]);
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointGetUniversalAnchor(jointId, pos);
                dJointSetUniversalAnchor(jointId, pos[0], pos[1], pos[2]);
                break;
            case JOINT_TYPE_FIXED:
                dJointDestroy(jointId);
                jointId = dJointCreateFixed(theWorld->getWorld(), 0);
                dJointAttach(jointId, body1, body2);
                dJointSetFixed(jointId);
                dJointSetFeedback(jointId, &feedback);
                // used for the integration study of the SpaceClimber
                //dJointSetFixedParam(jointId, dParamCFM, cfm1);//0.0002);
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
        }

        /**
         * \brief Return the torque vector for the first axis of the joint
         *
         *     v1[0] = b1_pos[0] - anchor[0];
         *     v1[1] = b1_pos[1] - anchor[1];
         *     v1[2] = b1_pos[2] - anchor[2];
         *     radius = dSqrt(v1[0]*v1[0]+
         *                    v1[1]*v1[1]+
         *                    v1[2]*v1[2]);
         *     normal[0] =  axis[1]*v1[2] - axis[2]*v1[1];
         *     normal[1] = -axis[0]*v1[2] + axis[2]*v1[0];
         *     normal[2] =  axis[0]*v1[1] - axis[1]*v1[0];
         *     dot = (normal[0]*feedback.f1[0]+
         *            normal[1]*feedback.f1[1]+
         *            normal[2]*feedback.f1[2]);
         *     normal[0] *= dot*radius;
         *     normal[1] *= dot*radius;
         *     normal[2] *= dot*radius;
         */
        void Joint::getAxisTorque(Vector *t) const
        {
            t->x() = axis1_torque.x();
            t->y() = axis1_torque.y();
            t->z() = axis1_torque.z();
        }

        void Joint::getAxis2Torque(Vector *t) const
        {
            t->x() = axis2_torque.x();
            t->y() = axis2_torque.y();
            t->z() = axis2_torque.z();
        }

        /**
         * \brief we need to calculate the axis torques and joint load before
         * we can return it.
         *
         */
        void Joint::update(void)
        {
            const dReal *b1_pos, *b2_pos;
            dReal anchor[4], axis[4], axis2[4];
            int calc1 = 0, calc2 = 0;
            dReal radius, dot, torque;
            dReal v1[3], normal[3], load[3], tmp1[3], axis_force[3];
            //MutexLocker locker(&(theWorld->iMutex));

            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                dJointGetHingeAnchor(jointId, anchor);
                dJointGetHingeAxis(jointId, axis);
                calc1 = 1;
                break;
            case JOINT_TYPE_HINGE2:
                dJointGetHinge2Anchor(jointId, anchor);
                dJointGetHinge2Axis1(jointId, axis);
                dJointGetHinge2Axis2(jointId, axis2);
                calc1 = 1;
                calc2 = 0;
                break;
            case JOINT_TYPE_SLIDER:
                dJointGetSliderAxis(jointId, axis);
                calc1 = 2;
                break;
            case JOINT_TYPE_BALL:
                // no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                dJointGetUniversalAnchor(jointId, anchor);
                dJointGetUniversalAxis1(jointId, axis);
                dJointGetUniversalAxis2(jointId, axis2);
                calc1 = 1;
                calc2 = 0;
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
            motor_torque = -feedback.lambda;
            axis1_torque.x() = axis1_torque.y() = axis1_torque.z() = 0;
            axis2_torque.x() = axis2_torque.y() = axis2_torque.z() = 0;
            joint_load.x() = joint_load.y() = joint_load.z() = 0;
            if(calc1 == 1)
            {
                if(body1)
                {
                    b1_pos = dBodyGetPosition(body1);
                    dOP(v1, -, b1_pos, anchor);
                    //radius = dLENGTH(v1);
                    dCROSS(normal, =, axis, v1);
                    dot = dDOT(normal, feedback.f1);
                    dOPEC(normal, *=, dot);
                    dOP(load, -, feedback.f1, normal);
                    dCROSS(tmp1, =, v1, normal);
                    axis1_torque.x() = (sReal)tmp1[0];
                    axis1_torque.y() = (sReal)tmp1[1];
                    axis1_torque.z() = (sReal)tmp1[2];
                    dCROSS(tmp1, =, v1, load);
                    joint_load.x() = (sReal)tmp1[0];
                    joint_load.y() = (sReal)tmp1[1];
                    joint_load.z() = (sReal)tmp1[2];
                    // now nearly the same for the torque
                    dot = dDOT(axis, feedback.t1);
                    dOPC(tmp1, *, axis, dot);
                    dOP(load, -, feedback.t1, tmp1);
                    axis1_torque.x() += (sReal)tmp1[0];
                    axis1_torque.y() += (sReal)tmp1[1];
                    axis1_torque.z() += (sReal)tmp1[2];
                    joint_load.x() += (sReal)load[0];
                    joint_load.y() += (sReal)load[1];
                    joint_load.z() += (sReal)load[2];
                } else if(body2)
                {
                    // now we do it correct
                    // first get the position vector v1
                    b2_pos = dBodyGetPosition(body2);
                    dOP(v1, -, b2_pos, anchor);

                    // then differentiate the torque from the force
                    dot = dDOT(feedback.f2, v1) / dDOT(v1, v1);
                    dOPC(axis_force, *, v1, dot);

                    // the difference is the torque
                    dOP(tmp1, -, feedback.f2, axis_force);

                    // the torque value is given by:
                    torque = dLENGTH(tmp1) / dLENGTH(v1);
                    // then get the normal to v1 and the torque vector
                    dCROSS(normal, =, v1, tmp1);
                    // and scale the normal to represent the correct torque length
                    dOPEC(normal, *=, torque / dLENGTH(normal));
                    // now the normal represents the torque vector at the anchor
                    // and we make a projection to the axis
                    dot = dDOT(normal, axis);
                    dOPC(tmp1, *, axis, dot);
                    dOP(load, -, normal, tmp1);
                    //dCROSS(tmp1, =, v1, normal);
                    axis1_torque.x() = (sReal)tmp1[0];
                    axis1_torque.y() = (sReal)tmp1[1];
                    axis1_torque.z() = (sReal)tmp1[2];
                    //dCROSS(tmp1, =, v1, load);
                    joint_load.x() = (sReal)load[0];
                    joint_load.y() = (sReal)load[1];
                    joint_load.z() = (sReal)load[2];
                    // now nearly the same for the torque
                    dot = dDOT(feedback.t2, axis);
                    dOPC(tmp1, *, axis, dot);
                    dOP(load, -, feedback.t2, tmp1);
                    //axis1_torque.x() += (sReal)tmp1[0];
                    //axis1_torque.y() += (sReal)tmp1[1];
                    //axis1_torque.z() += (sReal)tmp1[2];
                    joint_load.x() += (sReal)load[0];
                    joint_load.y() += (sReal)load[1];
                    joint_load.z() += (sReal)load[2];
                }
            } else if(calc1 == 2)
            {
                // this is for the slider
            }
            if(calc2 == 1)
            {
                if(body1)
                {
                    b1_pos = dBodyGetPosition(body1);
                    dOP(v1, -, b1_pos, anchor);
                    radius = dLENGTH(v1);
                    dCROSS(normal, =, axis2, v1);
                    dot = dDOT(normal, feedback.f1);
                    dOPEC(normal, *=, dot*radius);
                    axis2_torque.x() = (sReal)normal[0];
                    axis2_torque.y() = (sReal)normal[1];
                    axis2_torque.z() = (sReal)normal[2];
                    // now nearly the same for the torque
                    dot = dDOT(axis2, feedback.t1);
                    axis2_torque.x() += (sReal)(axis2[0]*dot);
                    axis2_torque.y() += (sReal)(axis2[1]*dot);
                    axis2_torque.z() += (sReal)(axis2[2]*dot);
                }
                if(body2)
                {
                    b2_pos = dBodyGetPosition(body2);
                    dOP(v1, -, b2_pos, anchor);
                    radius = dLENGTH(v1);
                    dCROSS(normal, =, axis2, v1);
                    dot = dDOT(normal, feedback.f2);
                    dOPEC(normal, *=, dot*radius);
                    axis2_torque.x() += (sReal)normal[0];
                    axis2_torque.y() += (sReal)normal[1];
                    axis2_torque.z() += (sReal)normal[2];
                    // now nearly the same for the torque
                    dot = dDOT(axis2, feedback.t2);
                    axis2_torque.x() += (sReal)(axis2[0]*dot);
                    axis2_torque.y() += (sReal)(axis2[1]*dot);
                    axis2_torque.z() += (sReal)(axis2[2]*dot);
                }
            }
        }

        void Joint::getJointLoad(Vector *t) const
        {
            t->x() = joint_load.x();
            t->y() = joint_load.y();
            t->z() = joint_load.z();
        }

        sReal Joint::getVelocity(void) const
        {
            MutexLocker locker(&(theWorld->iMutex));
            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                return -1*(sReal)dJointGetHingeAngleRate(jointId);
                break;
            case JOINT_TYPE_HINGE2:
                return (sReal)dJointGetHinge2Angle1Rate(jointId);
                break;
            case JOINT_TYPE_SLIDER:
                return -1*(sReal)dJointGetSliderPositionRate(jointId);
                break;
            case JOINT_TYPE_BALL:
                // no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                return (sReal)dJointGetUniversalAngle1Rate(jointId);
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
            return 0;
        }

        sReal Joint::getVelocity2(void) const
        {
            MutexLocker locker(&(theWorld->iMutex));
            switch(joint_type)
            {
            case  JOINT_TYPE_HINGE:
                break;
            case JOINT_TYPE_HINGE2:
                return (sReal)dJointGetHinge2Angle2Rate(jointId);
                break;
            case JOINT_TYPE_SLIDER:
                break;
            case JOINT_TYPE_BALL:
                // no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                return (sReal)dJointGetUniversalAngle2Rate(jointId);
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
            return 0;
        }

        void Joint::setTorque(sReal torque)
        {
            MutexLocker locker(&(theWorld->iMutex));
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                dJointAddHingeTorque(jointId, -1*torque);
                break;
            case JOINT_TYPE_HINGE2:
                break;
            case JOINT_TYPE_SLIDER:
                dJointAddSliderForce(jointId, -1*torque);
                break;
            case JOINT_TYPE_BALL:
                // no axis
                break;
            case JOINT_TYPE_UNIVERSAL:

                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
        }

        void Joint::setTorque2(sReal torque)
        {
            CPP_UNUSED(torque);
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                break;
            case JOINT_TYPE_HINGE2:
                break;
            case JOINT_TYPE_SLIDER:
                break;
            case JOINT_TYPE_BALL:
                // no axis
                break;
            case JOINT_TYPE_UNIVERSAL:
                break;
            default:
                // no correct type is specified, so no physically node will be created
                break;
            }
        }

        void Joint::changeStepSize(const JointData &jointS)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(theWorld && theWorld->existsWorld())
            {
                calculateCfmErp();

                switch(jointS.type)
                {
                case  JOINT_TYPE_HINGE:
                    if(damping>0.00000001)
                    {
                        dJointSetHingeParam(jointId, dParamFMax, damping);
                        dJointSetHingeParam(jointId, dParamVel, 0);
                    }
                    if(spring > 0.00000001)
                    {
                        dJointSetHingeParam(jointId, dParamLoStop, lo1);
                        dJointSetHingeParam(jointId, dParamHiStop, hi1);
                        dJointSetHingeParam(jointId, dParamStopCFM, cfm1);
                        dJointSetHingeParam(jointId, dParamStopERP, erp1);
                    } else if(lo1 != 0)
                    {
                        dJointSetHingeParam(jointId, dParamLoStop, lo1);
                        dJointSetHingeParam(jointId, dParamHiStop, hi1);
                    }
                    break;
                case JOINT_TYPE_HINGE2:
                    if(damping>0.00000001)
                    {
                        dJointSetHinge2Param(jointId, dParamFMax, damping);
                        dJointSetHinge2Param(jointId, dParamVel, 0);
                        dJointSetHinge2Param(jointId, dParamFMax2, damping);
                        dJointSetHinge2Param(jointId, dParamVel2, 0);
                    }
                    if(spring > 0.00000001)
                    {
                        dJointSetHinge2Param(jointId, dParamLoStop, lo1);
                        dJointSetHinge2Param(jointId, dParamHiStop, hi1);
                        dJointSetHinge2Param(jointId, dParamLoStop2, lo2);
                        dJointSetHinge2Param(jointId, dParamHiStop2, hi2);
                        dJointSetHinge2Param(jointId, dParamStopCFM, cfm1);
                        dJointSetHinge2Param(jointId, dParamStopERP, erp1);
                        dJointSetHinge2Param(jointId, dParamStopCFM2, cfm2);
                        dJointSetHinge2Param(jointId, dParamStopERP2, erp2);
                        dJointSetHinge2Param(jointId, dParamCFM, cfm);
                    }
                    break;
                case JOINT_TYPE_SLIDER:
                    if(damping > 0.00000001)
                    {
                        dJointSetSliderParam(jointId, dParamFMax, damping);
                        dJointSetSliderParam(jointId, dParamVel, 0);
                    }
                    if(spring > 0.00000001)
                    {
                        dJointSetSliderParam(jointId, dParamLoStop, lo1);
                        dJointSetSliderParam(jointId, dParamHiStop, hi1);
                        dJointSetSliderParam(jointId, dParamStopCFM, cfm1);
                        dJointSetSliderParam(jointId, dParamStopERP, erp1);
                    } else if(lo1 != 0)
                    {
                        dJointSetSliderParam(jointId, dParamLoStop, lo1);
                        dJointSetSliderParam(jointId, dParamHiStop, hi1);
                    }
                    break;
                case JOINT_TYPE_BALL:
                    /*
                      if(damping > 0.00000001) {
                      dJointSetAMotorParam(ball_motor, dParamVel, 0);
                      dJointSetAMotorParam(ball_motor, dParamVel2, 0);
                      dJointSetAMotorParam(ball_motor, dParamVel3, 0);
                      dJointSetAMotorParam(ball_motor, dParamFMax, damping);
                      dJointSetAMotorParam(ball_motor, dParamFMax2, damping);
                      dJointSetAMotorParam(ball_motor, dParamFMax3, damping);
                      }
                      else if(spring > 0.00000001) {
                      dJointSetAMotorParam(ball_motor, dParamLoStop, lo2);
                      dJointSetAMotorParam(ball_motor, dParamLoStop2, lo2);
                      dJointSetAMotorParam(ball_motor, dParamLoStop3, lo2);
                      dJointSetAMotorParam(ball_motor, dParamHiStop, hi1);
                      dJointSetAMotorParam(ball_motor, dParamHiStop2, hi2);
                      dJointSetAMotorParam(ball_motor, dParamHiStop3, hi2);
                      dJointSetAMotorParam(ball_motor, dParamCFM, cfm1);
                      dJointSetAMotorParam(ball_motor, dParamCFM2, cfm2);
                      dJointSetAMotorParam(ball_motor, dParamCFM3, cfm2);
                      dJointSetAMotorParam(ball_motor, dParamERP, erp1);
                      dJointSetAMotorParam(ball_motor, dParamERP2, erp2);
                      dJointSetAMotorParam(ball_motor, dParamERP3, erp2);
                      }
                    */
                    break;
                case JOINT_TYPE_UNIVERSAL:
                    if(damping>0.00000001)
                    {
                        dJointSetUniversalParam(jointId, dParamFMax, damping);
                        dJointSetUniversalParam(jointId, dParamVel, 0);
                        dJointSetUniversalParam(jointId, dParamFMax2, damping);
                        dJointSetUniversalParam(jointId, dParamVel2, 0);
                    }
                    if(spring > 0.00000001)
                    {
                        dJointSetUniversalParam(jointId, dParamLoStop, lo1);
                        dJointSetUniversalParam(jointId, dParamHiStop, hi1);
                        dJointSetUniversalParam(jointId, dParamLoStop2, lo2);
                        dJointSetUniversalParam(jointId, dParamHiStop2, hi2);
                        dJointSetUniversalParam(jointId, dParamStopCFM, cfm1);
                        dJointSetUniversalParam(jointId, dParamStopERP, erp1);
                        dJointSetUniversalParam(jointId, dParamStopCFM2, cfm2);
                        dJointSetUniversalParam(jointId, dParamStopERP2, erp2);
                        dJointSetUniversalParam(jointId, dParamCFM, cfm);
                    }
                    break;
                default:
                    // no correct type is specified, so no physically node will be created
                    break;
                }
            }
        }

        sReal Joint::getMotorTorque(void) const
        {
            return motor_torque;
        }

        interfaces::sReal Joint::getLowStop() const
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                return -1*dJointGetHingeParam(jointId, dParamHiStop);
            case JOINT_TYPE_HINGE2:
                return dJointGetHinge2Param(jointId, dParamLoStop);
            case JOINT_TYPE_SLIDER:
                return -1*dJointGetSliderParam(jointId, dParamHiStop);
            default:
                // not implemented yes
                fprintf(stderr, "mars::ode_physics::Joint: getLowStop for type %d not implemented yet\n", joint_type);
                return 0.;
            }
        }

        interfaces::sReal Joint::getHighStop() const
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                return -1*dJointGetHingeParam(jointId, dParamLoStop);
            case JOINT_TYPE_HINGE2:
                return dJointGetHinge2Param(jointId, dParamHiStop);
            case JOINT_TYPE_SLIDER:
                return -1*dJointGetSliderParam(jointId, dParamLoStop);
            default:
                // not implemented yes
                fprintf(stderr, "mars::ode_physics::Joint: getHighStop for type %d not implemented yet\n", joint_type);
                return 0.;
            }
        }

        interfaces::sReal Joint::getLowStop2() const
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE2:
                return dJointGetHinge2Param(jointId, dParamLoStop2);
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: getLowStop2 for type %d not implemented yet\n", joint_type);
                return 0.;
            }
        }

        interfaces::sReal Joint::getHighStop2() const
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE2:
                return dJointGetHinge2Param(jointId, dParamHiStop2);
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: getHighStop2 for type %d not implemented yet\n", joint_type);
                return 0.;
            }
        }

        interfaces::sReal Joint::getCFM() const
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                return dJointGetHingeParam(jointId, dParamCFM);
            case JOINT_TYPE_HINGE2:
                return dJointGetHinge2Param(jointId, dParamCFM);
            case JOINT_TYPE_SLIDER:
                return dJointGetSliderParam(jointId, dParamCFM);
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: cfm for type %d not implemented yet\n", joint_type);
                return 0.;
            }
        }

        void Joint::setLowStop(interfaces::sReal lowStop)
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                dJointSetHingeParam(jointId, dParamLoStop, lowStop);
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamLoStop, lowStop);
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderParam(jointId, dParamLoStop, lowStop);
                break;
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: setLowStop for type %d not implemented yet\n", joint_type);
                break;
            }
        }

        void Joint::setHighStop(interfaces::sReal highStop)
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                dJointSetHingeParam(jointId, dParamLoStop, -1*highStop);
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamHiStop, highStop);
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderParam(jointId, dParamLoStop, -1*highStop);
                break;
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: setHighStop for type %d not implemented yet\n", joint_type);
                break;
            }
        }

        void Joint::setLowStop2(interfaces::sReal lowStop)
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamHiStop, -1*lowStop);
                break;
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: setLowStop2 for type %d not implemented yet\n", joint_type);
                break;
            }
        }

        void Joint::setHighStop2(interfaces::sReal highStop)
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamLoStop2, -1*highStop);
                break;
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: setHighStop2 for type %d not implemented yet\n", joint_type);
                break;
            }
        }

        void Joint::setCFM(interfaces::sReal cfm)
        {
            switch(joint_type)
            {
            case JOINT_TYPE_HINGE:
                dJointSetHingeParam(jointId, dParamCFM, cfm);
                break;
            case JOINT_TYPE_HINGE2:
                dJointSetHinge2Param(jointId, dParamCFM, cfm);
                break;
            case JOINT_TYPE_SLIDER:
                dJointSetSliderParam(jointId, dParamCFM, cfm);
                break;
            default:
                // not implemented yet
                fprintf(stderr, "mars::ode_physics::Joint: cfm for type %d not implemented yet\n", joint_type);
                break;
            }
        }

        void Joint::getDataBrokerNames(std::string& groupName, std::string& dataName)
        {
            groupName = std::string{"mars_sim"};
            dataName = std::string{"Joints/"} + name;
        }


        // ## ConfigMapInterface methods ##
        configmaps::ConfigMap Joint::getConfigMap() const
        {
            configmaps::ConfigMap result;
            result.append(config);

            // axis1 contains axis in global coordinates
            result.erase("axis1");

            mars::utils::Vector axis;
            getAxis(&axis);
            result["axis"] = mars::utils::vectorToConfigItem(axis);

            mars::utils::Vector anchor;
            getAnchor(&anchor);
            result["anchor"] = mars::utils::vectorToConfigItem(anchor);
            return result;
        }

        std::vector<std::string> Joint::getEditPattern(const std::string& basePath) const
        {
            // empty vector would result in all entries being editable
            return std::vector<std::string>{""};
        }

        // TODO: Is everything previously available in JointManager::edit covered?
        // TODO (old, moved from JointManager): do we need to edit angle offsets
        void Joint::edit(const std::string& configPath, const std::string& value)
        {
            const std::vector<std::string> configPathTokens{mars::utils::explodeString('/', configPath)};
            const size_t n = configPathTokens.size();
            if(configPathTokens[n-2] == "axis")
            {
                const double v = std::stod(value);
                mars::utils::Vector axis;
                getAxis(&axis);
                if(configPathTokens[n-1] == "x") axis.x() = v;
                else if(configPathTokens[n-1] == "y") axis.y() = v;
                else if(configPathTokens[n-1] == "z") axis.z() = v;
                setAxis(axis);
            } else if(configPathTokens[n-2] == "axis2")
            {
                const double v = std::stod(value);
                mars::utils::Vector axis2;
                getAxis2(&axis2);
                if(configPathTokens[n-1] == "x") axis2.x() = v;
                else if(configPathTokens[n-1] == "y") axis2.y() = v;
                else if(configPathTokens[n-1] == "z") axis2.z() = v;
                setAxis2(axis2);
            } else if(configPathTokens[n-2] == "anchor")
            {
                const double v = std::stod(value);
                mars::utils::Vector anchor;
                getAnchor(&anchor);
                if(configPathTokens[n-1] == "x") anchor.x() = v;
                else if(configPathTokens[n-1] == "y") anchor.y() = v;
                else if(configPathTokens[n-1] == "z") anchor.z() = v;
                setAnchor(anchor);
            }
        }

        // ## DataBroker methods ##
        void Joint::addToDataBroker()
        {
            if(dataBroker)
            {
                std::string groupName, dataName;
                groupName = "mars_sim";
                dataName = "Joints/"+name;
                // initialize the dataBroker Package
                data_broker::DataPackage dbPackage;
                dbPackageMapping.writePackage(&dbPackage);
                dataBroker->pushData(groupName, dataName, dbPackage, NULL,
                                     data_broker::DATA_PACKAGE_READ_FLAG);
                // register as producer
                dataBroker->registerTimedProducer(this, groupName, dataName,
                                                  "mars_sim/simTimer", 0);
            }
        }

        void Joint::removeFromDataBroker()
        {
            if(dataBroker)
            {
                std::string groupName, dataName;
                groupName = "mars_sim";
                dataName = name;
                dataBroker->unregisterTimedProducer(this, groupName, dataName,
                                                    "mars_sim/simTimer");
            }
        }

        void Joint::produceData(const data_broker::DataInfo &info,
                                data_broker::DataPackage *dbPackage,
                                int callbackParam)
        {
            updateState();
            dbPackageMapping.writePackage(dbPackage);
        }
    } // end of namespace ode_physics
} // end of namespace mars
