//TODO cleanup includes
#include "Frame.hpp"
#include "Object.hpp"

#include <mars_interfaces/Logging.hpp>
#include <mars_utils/MutexLocker.h>
#include <mars_utils/mathUtils.h>

#include <cmath>
#include <set>
#include <cstdlib>
#include <stdexcept> 

#include <ode/odemath.h>

#include <iostream>
using namespace configmaps;
namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;

        /**
         * \brief Creates a empty frame object.
         *
         * \pre
         *     - the pointer to the physics Interface should be correct.
         *       This implementation can be a bad trap. The class that implements the
         *       physics interface, have to inherit from the interface at first,
         *       otherwise this implementation will cause bad error while pointing
         *       an incorrect adresses.
         *
         * \post
         *     - the class should have saved the pointer to the physics implementation
         *     - the body and geom should be initialized to 0
         */
        Frame::Frame(PhysicsInterface *world,
                     data_broker::DataBrokerInterface *dataBroker,
                     ConfigMap &config) : dataBroker(dataBroker),
                                          position(0.0, 0.0, 0.0),
                                          q(1.0, 0.0, 0.0, 0.0),
                                          ground_contact{false}
        {
            theWorld = dynamic_cast<WorldPhysics*>(world);
            nBody = 0;
            dMassSetZero(&nMass);
            contactForce = 0;
            contactForceVector = {0,0,0};

            offsetPos = Vector(0, 0, 0);
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

            // TODO: give config to odeFrame to allow to configure pushToDataBroker
            // TODO: move this setup to method so that it can changed/reconfigured online
            if(pushToDataBroker > 0)
            {
                //dbPackageMapping.add("id", &sNode.index);
                dbPackageMapping.add("position/x", &position.x());
                dbPackageMapping.add("position/y", &position.y());
                dbPackageMapping.add("position/z", &position.z());
                dbPackageMapping.add("rotation/x", &q.x());
                dbPackageMapping.add("rotation/y", &q.y());
                dbPackageMapping.add("rotation/z", &q.z());
                dbPackageMapping.add("rotation/w", &q.w());
                dbPackageMapping.add("contact", &ground_contact);
                dbPackageMapping.add("contactForce/norm", &contactForce);
            }
            if(pushToDataBroker > 1)
            {
                dbPackageMapping.add("linearVelocity/x", &linearVelocity.x());
                dbPackageMapping.add("linearVelocity/y", &linearVelocity.y());
                dbPackageMapping.add("linearVelocity/z", &linearVelocity.z());
                dbPackageMapping.add("angularVelocity/x", &angularVelocity.x());
                dbPackageMapping.add("angularVelocity/y", &angularVelocity.y());
                dbPackageMapping.add("angularVelocity/z", &angularVelocity.z());
                dbPackageMapping.add("linearAcceleration/x", &linearAcceleration.x());
                dbPackageMapping.add("linearAcceleration/y", &linearAcceleration.y());
                dbPackageMapping.add("linearAcceleration/z", &linearAcceleration.z());
                dbPackageMapping.add("angularAcceleration/x", &angularAcceleration.x());
                dbPackageMapping.add("angularAcceleration/y", &angularAcceleration.y());
                dbPackageMapping.add("angularAcceleration/z", &angularAcceleration.z());
                dbPackageMapping.add("force/x", &force.x());
                dbPackageMapping.add("force/y", &force.y());
                dbPackageMapping.add("force/z", &force.z());
                dbPackageMapping.add("torque/x", &torque.x());
                dbPackageMapping.add("torque/y", &torque.y());
                dbPackageMapping.add("torque/z", &torque.z());
                dbPackageMapping.add("contactForce/x", &contactForceVector.x());
                dbPackageMapping.add("contactForce/y", &contactForceVector.y());
                dbPackageMapping.add("contactForce/z", &contactForceVector.z());                
            }
            if(pushToDataBroker > 0)
            {
                addToDataBroker();
            }
        }

        /**
         * \brief Destroys the node in the physical world.
         *
         * pre:
         *     - theWorld is the correct world object
         *
         * post:
         *     - all physical representation of the node should be cleared
         *
         * are the geom and the body realy all thing to take care of?
         */
        Frame::~Frame(void)
        {
            //std::vector<sensor_list_element>::iterator iter;
            const MutexLocker locker{&(theWorld->iMutex)};

            if(nBody)
            {
                dBodyDestroy(nBody);
            }
            removeFromDataBroker();

            // TODO: how does this loop work? why doesn't it run forever?
            // for(iter = sensor_list.begin(); iter != sensor_list.end();) {
            //     if((*iter).gd){
            //         delete ((*iter).gd);
            //         (*iter).gd = 0;
            //     }
            //     dGeomDestroy((*iter).geom);
            //     sensor_list.erase(iter);
            // }
        }

        const std::string& Frame::getName() const
        {
            return name;
        }

        void Frame::computeContactForce()
        {
            utils::Vector resultantForce = {0,0,0};

            for (const auto& jointFeedback : jointFeedbacks){
                resultantForce.x() += jointFeedback->f1[0];
                resultantForce.y() += jointFeedback->f1[1];
                resultantForce.z() += jointFeedback->f1[2];
            }
            
            contactForceVector = std::move(resultantForce);
            contactForce = contactForceVector.norm();
        }

        const utils::Vector& Frame::getContactForceVector() const
        {
            return contactForceVector;
        }

        const sReal& Frame::getContactForce() const
        {
            return contactForce;
        }

        void Frame::addObject(Object *object)
        {
            auto it = std::find(objects.begin(), objects.end(), object);
            if(it != objects.end())
            {
                // TODO: add proper error handling
            }
            else
            {
                objects.push_back(object);
                if(nBody == NULL)
                {
                    nBody = dBodyCreate(theWorld->getWorld());
                    dBodySetPosition(nBody, (dReal)position.x(), (dReal)position.y(), (dReal)position.z());
                    dQuaternion tmp;
                    tmp[1] = (dReal)q.x();
                    tmp[2] = (dReal)q.y();
                    tmp[3] = (dReal)q.z();
                    tmp[0] = (dReal)q.w();
                    dBodySetQuaternion(nBody, tmp);
                }
                // TODO: add object inertial to nBody
                addObjectMass(object);
            }
        }


        void Frame::removeObject(Object* object)
        {
            MutexLocker locker(&(theWorld->iMutex));
            // if this is only object: destroy nBody
            // otherwise remove object mass/interia from nBody
            for(auto it=objects.begin(); it!=objects.end(); ++it)
            {
                if(*it == object)
                {
                    objects.erase(it);
                    // TODO: remove inertial from nMass
                    break;
                }
            }
            if(objects.size() ==  0)
            {
                if(nBody) dBodyDestroy(nBody);
                nBody = 0;
                dMassSetZero(&nMass);
            }
        }


        void Frame::getPosition(Vector* pos) const {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody)
            {
                dReal tpos[3];
                dReal massOffset[3];
                massOffset[0] = offsetPos.x();
                massOffset[1] = offsetPos.y();
                massOffset[2] = offsetPos.z();

                // the mass displacement is in bodyframe
                // to get the displacement in world coordinates
                // we have to rotate the vector
                dMatrix3 R;
                const dReal *brot = dBodyGetQuaternion(nBody);
                dRfromQ(R, brot);
                dMULTIPLY0_331(tpos, R, massOffset);
                const dReal *bpos = dBodyGetPosition(nBody);

                tpos[0] -= bpos[0];
                tpos[1] -= bpos[1];
                tpos[2] -= bpos[2];

                pos->x() = -(sReal)tpos[0];
                pos->y() = -(sReal)tpos[1];
                pos->z() = -(sReal)tpos[2];
            } else
            {
                pos->x() = 0;
                pos->y() = 0;
                pos->z() = 0;
            }
        }

        /**
         * \brief The method sets the position of the physical node model to the
         * position of the param.  If move_group is set, all nodes of a composite
         * group will be moved, otherwise only this node will be moved.  A vector
         * from the old position to the new will be returned.
         *
         * I don't know if we should use this function in a way like it is
         * implemented now. The pre and post conditions could loke like this:
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the pos param should point to a correct position struct
         *
         * post:
         *     - if there is a physically representation and the node is movable
         *       set the position of the corresponding body to the given parameter
         *     - otherwise, we have to do nothing
         */
        void Frame::setPosition(const Vector &pos)
        {
            const dReal *tpos;
            const dReal *tpos2;
            dReal npos[3];
            Vector offset;
            MutexLocker locker(&(theWorld->iMutex));

            position = pos;
            if(nBody)
            {
                dReal tpos[3];
                dReal massOffset[3];
                massOffset[0] = offsetPos.x();
                massOffset[1] = offsetPos.y();
                massOffset[2] = offsetPos.z();

                // the mass displacement is in bodyframe
                // to get the displacement in world coordinates
                // we have to rotate the vector
                dMatrix3 R;
                const dReal *brot = dBodyGetQuaternion(nBody);
                dRfromQ(R, brot);
                dMULTIPLY0_331(tpos, R, massOffset);
                // tpos = dBodyGetPosition(nBody);
                // offset.x() = pos.x() - (sReal)(tpos[0]);
                // offset.y() = pos.y() - (sReal)(tpos[1]);
                // offset.z() = pos.z() - (sReal)(tpos[2]);
                dBodySetPosition(nBody, (dReal)pos.x()+tpos[0], (dReal)pos.y()+tpos[1], (dReal)pos.z()+tpos[2]);
            }
        }

        /**
         * \brief The method copies the Quaternion of the physically node at the
         * adress of the Quaternion pointer q.
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the node should be movable
         *     - the q param should point to a correct Quaternion struct
         *
         * post:
         *     - if there is a physical representation and the node is movable
         *       the Quaternion struct should be filled with the physical rotation
         *       of the node
         *     - otherwise a standard return of zero rotation should be set
         */
        void Frame::getRotation(Quaternion* q) const
        {
            dQuaternion tmp;
            MutexLocker locker(&(theWorld->iMutex));

            if(nBody)
            {
                const dReal *rot = dBodyGetQuaternion(nBody);
                q->x() = (sReal)rot[1];
                q->y() = (sReal)rot[2];
                q->z() = (sReal)rot[3];
                q->w() = (sReal)rot[0];
            } else
            {
                *q = this->q;
            }
        }


        void Frame::getPose(utils::Vector *position, utils::Quaternion *rotation) const
        {
            getPosition(position);
            getRotation(rotation);
        }

        void Frame::setPose(const utils::Vector& position, const utils::Quaternion& rotation, const bool reset_velocities)
        {
            // TODO: explain why rotation has to be set first!
            setRotation(rotation);
            setPosition(position);

            if(reset_velocities)
            {
                const auto zero = utils::Vector{.0, .0, .0};
                setLinearVelocity(zero);
                setAngularVelocity(zero);
            }
        }

        /**
         * \brief The method copies the linear velocity of the physically node at the
         * adress of the linear_vel pointer vel.
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the node should be movable
         *     - the vel param should point to a correct linear_vel struct
         *
         * post:
         *     - if there is a physical representation and the node is movable
         *       the linear_vel struct should be filled with the physical linar
         *       velocity of the node
         *     - otherwise a standard return of zero velocity should be set
         */
        void Frame::getLinearVelocity(Vector* vel) const
        {
            const dReal *tmp;
            MutexLocker locker(&(theWorld->iMutex));

            if(nBody)
            {
                tmp = dBodyGetLinearVel(nBody);
                vel->x() = (sReal)tmp[0];
                vel->y() = (sReal)tmp[1];
                vel->z() = (sReal)tmp[2];
            } else
            {
                vel->x() = (sReal)0;
                vel->y() = (sReal)0;
                vel->z() = (sReal)0;
            }
        }

        /**
         * \brief The method copies the angular velocity of the physically node at the
         * adress of the angular_vel pointer vel.
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the node should be movable
         *     - the vel param should point to a correct angular_vel struct
         *
         * post:
         *     - if there is a physical representation and the node is movable
         *       the angular_vel struct should be filled with the physical angular
         *       velocity of the node
         *     - otherwise a standard return of zero velocity should be set
         */
        void Frame::getAngularVelocity(Vector* vel) const
        {
            const dReal *tmp;
            MutexLocker locker(&(theWorld->iMutex));

            if(nBody)
            {
                tmp = dBodyGetAngularVel(nBody);
                vel->x() = (sReal)tmp[0];
                vel->y() = (sReal)tmp[1];
                vel->z() = (sReal)tmp[2];
            } else
            {
                vel->x() = (sReal)0;
                vel->y() = (sReal)0;
                vel->z() = (sReal)0;
            }
        }

        /**
         * \brief The method copies the force of the physically node at the
         * adress of the force pointer force.
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the node should be movable
         *     - the f param should point to a correct force struct
         *
         * post:
         *     - if there is a physical representation and the node is movable
         *       the force struct should be filled with the physical
         *       force of the node
         *     - otherwise a standard return of zero force should be set
         */
        void Frame::getForce(Vector* f) const
        {
            const dReal *tmp;
            MutexLocker locker(&(theWorld->iMutex));

            if(nBody)
            {
                tmp = dBodyGetForce(nBody);
                f->x() = (sReal)tmp[0];
                f->y() = (sReal)tmp[1];
                f->z() = (sReal)tmp[2];
            } else
            {
                f->x() = (sReal)0;
                f->y() = (sReal)0;
                f->z() = (sReal)0;
            }
        }

        /**
         * \brief The method copies the torque of the physically node at the
         * adress of the torque pointer force.
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the node should be movable
         *     - the t param should point to a correct torque struct
         *
         * post:
         *     - if there is a physical representation and the node is movable
         *       the torque struct should be filled with the physical torque
         *       of the node
         *     - otherwise a standard return of zero torque should be set
         */
        void Frame::getTorque(Vector *t) const
        {
            const dReal *tmp;
            MutexLocker locker(&(theWorld->iMutex));

            if(nBody)
            {
                tmp = dBodyGetTorque(nBody);
                t->x() = (sReal)tmp[0];
                t->y() = (sReal)tmp[1];
                t->z() = (sReal)tmp[2];
            } else
            {
                t->x() = (sReal)0;
                t->y() = (sReal)0;
                t->z() = (sReal)0;
            }
        }


        /**
         * \brief This method sets the rotation of the physically node.
         *
         * I don't if and how to use this function yet. ^-^
         * If we need it, the pre and post conditions are like them in the set
         * position method.
         */
        void Frame::setRotation(const Quaternion &q)
        {
            dQuaternion tmp, tmp2, tmp3, tmp4, tmp5;
            const dReal *brot, *bpos, *gpos;
            Quaternion q2;
            dMatrix3 R;
            dVector3 pos, new_pos, new2_pos;
            MutexLocker locker(&(theWorld->iMutex));

            this->q = q;

            pos[0] = pos[1] = pos[2] = 0;
            tmp[1] = (dReal)q.x();
            tmp[2] = (dReal)q.y();
            tmp[3] = (dReal)q.z();
            tmp[0] = (dReal)q.w();
            if(nBody)
            {
                dReal tpos[3], new_pos[3];
                dReal massOffset[3];
                massOffset[0] = offsetPos.x();
                massOffset[1] = offsetPos.y();
                massOffset[2] = offsetPos.z();

                // the mass displacement is in bodyframe
                // to get the displacement in world coordinates
                // we have to rotate the vector
                dMatrix3 R;
                const dReal *brot = dBodyGetQuaternion(nBody);
                // we need the inverse of the body rotation
                dRfromQ(R, brot);
                // massOffset in world frame of old rotation
                dMULTIPLY0_331(tpos, R, massOffset);
                // tpos is massOffset in world

                bpos = dBodyGetPosition(nBody);
                // reference body position: rotation_point
                new_pos[0] = bpos[0]-tpos[0];
                new_pos[1] = bpos[1]-tpos[1];
                new_pos[2] = bpos[2]-tpos[2];

                // bpos+tpos is current position in world
                // bpos is rotation position in world
                // now we want to remove brot_old and apply brot_new
                dQtoR(tmp, R);
                // massOffset in world frame with new rotation
                dMULTIPLY0_331(tpos, R, massOffset);

                new_pos[0] += tpos[0];
                new_pos[1] += tpos[1];
                new_pos[2] += tpos[2];
                dBodySetPosition(nBody, new_pos[0], new_pos[1], new_pos[2]);

                dBodySetQuaternion(nBody, tmp);
            }

            // TODO: check if tmp or tmp2 have to be inverted
            // only needed if we have to return the offset rotation
            // dQMultiply2(tmp3, tmp, tmp2);
            // q2.x() = (sReal)tmp3[1];
            // q2.y() = (sReal)tmp3[2];
            // q2.z() = (sReal)tmp3[3];
            // q2.w() = (sReal)tmp3[0];
        }

        // /**
        //  * \brief This function sets the pointer to the physical world object.
        //  *
        //  * I don't think that we need this function.
        //  */
        // void Frame::setWorldObject(std::shared_ptr<PhysicsInterface>  world) {
        //     theWorld = std::dynamic_pointer_cast<WorldPhysics>(world);
        // }

        /**
         *\brief return the body;
         * this function is created to make it possible to get the
         * body from joint physics
         *TO DO : test if the Node has a body
         */
        dBodyID Frame::getBody() const
        {
            return nBody;
        }

        dBodyID Frame::acquireBody()
        {
            if(!nBody)
            {
                LOG_WARN("Body of empty Frame (%s) is requiered -> phyiscs is create a small \"fake\" body with inertial for sphere of 0.01m radius and 0.01Kg mass!", name.c_str());
                nBody = dBodyCreate(theWorld->getWorld());
                dBodySetPosition(nBody, (dReal)position.x(), (dReal)position.y(), (dReal)position.z());
                dQuaternion tmp;
                tmp[1] = (dReal)q.x();
                tmp[2] = (dReal)q.y();
                tmp[3] = (dReal)q.z();
                tmp[0] = (dReal)q.w();
                dBodySetQuaternion(nBody, tmp);

                dMassSetSphereTotal(&nMass, 0.01, 0.01);
                dBodySetMass(nBody, &nMass);
            }
            return nBody;
        }

        /**
         * \brief executes an rotation at a given point and returns the
         * new position of the node
         *
         * pre:
         *
         * post:
         */
        const Vector Frame::rotateAtPoint(const Vector &rotation_point,
                                          const Quaternion &rotation,
                                          bool move_group)
        {
            dQuaternion tmp, tmp2, tmp3;
            const dReal *bpos, *gpos, *brot;
            dVector3 pos, new_pos;
            Vector npos;
            dMatrix3 R;
            MutexLocker locker(&(theWorld->iMutex));

            tmp[1] = (dReal)rotation.x();
            tmp[2] = (dReal)rotation.y();
            tmp[3] = (dReal)rotation.z();
            tmp[0] = (dReal)rotation.w();
            brot = dBodyGetQuaternion(nBody);
            dQMultiply0(tmp2, tmp, brot);
            // we have to rotate the body and return the new position of the geom
            dBodySetQuaternion(nBody, tmp2);
            dQtoR(tmp, R);
            bpos = dBodyGetPosition(nBody);
            pos[0] = bpos[0] - (dReal)rotation_point.x();
            pos[1] = bpos[1] - (dReal)rotation_point.y();
            pos[2] = bpos[2] - (dReal)rotation_point.z();
            dMULTIPLY0_331(new_pos, R, pos);
            npos.x() = new_pos[0] + (dReal)rotation_point.x();
            npos.y() = new_pos[1] + (dReal)rotation_point.y();
            npos.z() = new_pos[2] + (dReal)rotation_point.z();
            dBodySetPosition(nBody, (dReal)npos.x(), (dReal)npos.y(), (dReal)npos.z());

            // TODO: check what this function should do and correct the code
            return npos;
        }

        /**
         * \brief returns the ode mass object
         *
         *
         * pre:
         *
         * post:
         */
        dMass Frame::getMass(void) const
        {
            return nMass;
        }

        /**
         * \brief Sets the linear velocity of a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the linear velocity of the body should be set
         */
        void Frame::setLinearVelocity(const Vector &velocity)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody) dBodySetLinearVel(nBody, (dReal)velocity.x(),
                                        (dReal)velocity.y(), (dReal)velocity.z());
        }

        /**
         * \brief Sets the angular velocity of a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the angular velocity of the body should be set
         */
        void Frame::setAngularVelocity(const Vector &velocity)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody) dBodySetAngularVel(nBody, (dReal)velocity.x(),
                                         (dReal)velocity.y(), (dReal)velocity.z());
        }

        /**
         * \brief Sets the force of a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the force of the body should be set
         */
        void Frame::setForce(const Vector &f)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody) dBodySetForce(nBody, (dReal)f.x(),
                                    (dReal)f.y(), (dReal)f.z());
        }

        /**
         * \brief Sets the torque of a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the torque of the body should be set
         */
        void Frame::setTorque(const Vector &t) {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody) dBodySetTorque(nBody, (dReal)t.x(),
                                     (dReal)t.y(), (dReal)t.z());
        }

        /**
         * \brief Adds a off-center force to a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the force should be added to the body
         */
        void Frame::addForce(const Vector &f, const Vector &p)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody)
            {
                dBodyAddForceAtPos(nBody,
                                   (dReal)f.x(), (dReal)f.y(), (dReal)f.z(),
                                   (dReal)p.x(), (dReal)p.y(), (dReal)p.z());
            }
        }
        /**
         * \brief Adds a force to a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the force should be added to the body
         */
        void Frame::addForce(const Vector &f)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody)
            {
                dBodyAddForce(nBody, (dReal)f.x(), (dReal)f.y(), (dReal)f.z());
            }
        }

        /**
         * \brief Adds a torque to a node
         *
         * pre:
         *      - the node should have a body
         *
         * post:
         *      - the torque should be added to the body
         */
        void Frame::addTorque(const Vector &t)
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody) dBodyAddTorque(nBody, (dReal)t.x(), (dReal)t.y(), (dReal)t.z());
        }

        void Frame::addObjectMass(Object *object)
        {
            dReal tpos[3];
            dMass myMass = object->getMass();

            // first rotate and translate the mass and add it to the body
            // we get the rotation and translation relative to the body-frame
            // from the geom
            Vector pos;
            object->getPosition(&pos);
            Quaternion q(1.0, 0., 0., 0.);
            object->getRotation(&q);
            dReal odeQ[4] = {q.w(), q.x(), q.y(), q.z()};
            dMatrix3 R;
            dRfromQ(R, odeQ);

            dMassRotate(&myMass, R);
            dMassTranslate(&myMass, pos.x(), pos.y(), pos.z());
            fprintf(stderr, "addObjectMass: %g %g\n", nMass.mass, myMass.mass);
            dMassAdd(&nMass, &myMass);


            // we create a position offset due to center of masses
            // TODO: find a better solution for this issue
            // the offset is in bodyframe
            offsetPos.x() += nMass.c[0];
            offsetPos.y() += nMass.c[1];
            offsetPos.z() += nMass.c[2];

            // by convention the bodyMass center of mass have to be zero
            // by adding the object mass to the body mass the body center of mass is shifted
            // so we move the body position by com and clear the com of the bodyMass

            // the mass displacement is in bodyframe
            // to get the displacement in world coordinates
            // we have to rotate the vector
            const dReal *brot = dBodyGetQuaternion(nBody);
            dRfromQ(R, brot);
            dMULTIPLY0_331(tpos, R, nMass.c);
            const dReal *bpos = dBodyGetPosition(nBody);
            if(fabs(tpos[0]) > 0.000001 || fabs(tpos[1]) > 0.000001 || fabs(tpos[2]) > 0.000001)
            {
                fprintf(stderr, "--------+++++++++++++++**********\n");
            }

            tpos[0] += bpos[0];
            tpos[1] += bpos[1];
            tpos[2] += bpos[2];

            dBodySetPosition(nBody, tpos[0], tpos[1], tpos[2]);
            position.x() = tpos[0];
            position.y() = tpos[1];
            position.z() = tpos[2];

            dMassTranslate(&nMass, -nMass.c[0], -nMass.c[1], -nMass.c[2]);

            dBodySetMass(nBody, &nMass);
        }

        void Frame::getAbsMass(dMass *tMass) const
        {
            // no lock because physics internal functions get locked elsewhere
            const dReal *pos = dBodyGetPosition(nBody);
            const dReal *rot = dBodyGetRotation(nBody);

            *tMass = nMass;
            dMassRotate(tMass, rot);
            dMassTranslate(tMass, pos[0], pos[1], pos[2]);
        }

        void Frame::getMass(sReal *mass, sReal *inertia) const
        {
            if(mass) *mass = nMass.mass;
            if(inertia)
            {
                inertia[0] = nMass.I[0];
                inertia[1] = nMass.I[1];
                inertia[2] = nMass.I[2];
                inertia[3] = nMass.I[4];
                inertia[4] = nMass.I[5];
                inertia[5] = nMass.I[6];
                inertia[6] = nMass.I[8];
                inertia[7] = nMass.I[9];
                inertia[8] = nMass.I[10];
            }
        }

        void Frame::updateState()
        {
            MutexLocker locker(&(theWorld->iMutex));
            if(nBody)
            {
                const dReal* tmp = dBodyGetPosition(nBody);
                position.x() = (sReal)tmp[0];
                position.y() = (sReal)tmp[1];
                position.z() = (sReal)tmp[2];
                const dReal *rot = dBodyGetQuaternion(nBody);
                q.x() = (sReal)rot[1];
                q.y() = (sReal)rot[2];
                q.z() = (sReal)rot[3];
                q.w() = (sReal)rot[0];
            }
        }

        void Frame::clearContactData()
        {
            for(auto jointFeedback : jointFeedbacks)
            {
                delete jointFeedback;
            }
            ground_contact = false;
            jointFeedbacks.clear();
            contactForceVector = {0,0,0};
            contactForce = 0.0;
        }

        // ## DataBroker methods ##
        void Frame::addToDataBroker()
        {
            if(dataBroker)
            {
                std::string groupName, dataName;
                groupName = "mars_sim";
                dataName = "Frames/"+name;
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

        void Frame::removeFromDataBroker()
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

        void Frame::produceData(const data_broker::DataInfo &info,
                                   data_broker::DataPackage *dbPackage,
                                   int callbackParam)
        {
            updateState();
            dbPackageMapping.writePackage(dbPackage);
        }

        void Frame::addContact(const ContactData& contact)
        {
            // TODO: create contact joint
            // get second body:
            dBodyID nBody2 = 0;
            double invert = 1.0;

            if(contact.body2)
            {
                const auto* frame2 = static_cast<const Frame*>(contact.body2.get());
                // we don't know if this is body1 or body2 in the contact
                if(frame2 == this)
                {
                    frame2 = static_cast<const Frame*>(contact.body1.get());
                    invert = -1.0;
                }
                if(frame2)
                {
                    // check if the other frame is part of this physics world
                    const auto& test = theWorld->getFrame(frame2->getName());
                    if(test)
                    {
                        nBody2 = frame2->getBody();
                    }
                }
            }

            dContact c;
            // transfer contact parameters to ode contact information
            // TODO: allow to transfer dContact via Contact data directly
            c.geom.pos[0] = contact.pos.x();
            c.geom.pos[1] = contact.pos.y();
            c.geom.pos[2] = contact.pos.z();
            c.geom.normal[0] = contact.normal.x()*invert;
            c.geom.normal[1] = contact.normal.y()*invert;
            c.geom.normal[2] = contact.normal.z()*invert;
            // fprintf(stderr, "at: %g %g %g\n", c.geom.pos[0], c.geom.pos[1], c.geom.pos[2]);
            // fprintf(stderr, "n: %g %g %g\n", c.geom.normal[0], c.geom.normal[1], c.geom.normal[2]);
            // fprintf(stderr, "with: %g\n", c.geom.depth);
            // TODO: generate and transfer all possible data
            c.geom.depth = contact.depth;

            c.surface.mode = dContactSoftERP | dContactSoftCFM;
            c.surface.mode |= dContactApprox1;
            c.surface.mu = contact.c_params.friction1;
            c.surface.mu2 = contact.c_params.friction2;
            c.surface.rho = contact.c_params.rolling_friction;
            c.surface.rho2 = contact.c_params.rolling_friction2;
            c.surface.rhoN = contact.c_params.spinning_friction;
            c.surface.bounce = contact.c_params.bounce;
            c.surface.bounce_vel = contact.c_params.bounce_vel;
            c.surface.soft_cfm = contact.c_params.cfm;
            c.surface.soft_erp = contact.c_params.erp;
            c.surface.motion1 = contact.c_params.motion1;
            c.surface.motion2 = contact.c_params.motion2;
            // TODO: c.surface.motionN = ???
            // TODO: c.surface.slip1 = ??? fds1?
            // TODO: c.surface.slip2 = ??? fds2?

            if(c.surface.mu != c.surface.mu2)
            {
                c.surface.mode |= dContactMu2;
            }

            dJointID contactJoint = theWorld->createContact(c, nBody, nBody2);

            if(!contactJoint){
                LOG_WARN("Failed to create contact for (%s) because there was already a joint between the two bodies", this->getName().c_str());
                return;
            }

            auto* const fb = new dJointFeedback{};
            dJointSetFeedback(contactJoint, fb);
            jointFeedbacks.push_back(fb);
            ground_contact = true;
        }

        void Frame::addLinkedFrame(std::shared_ptr<DynamicObject> linked)
        {
            connectedFrames.push_back(linked);
        }

        bool Frame::isLinkedFrame(std::shared_ptr<DynamicObject> linked)
        {
            for(auto& frame: connectedFrames)
            {
                if(frame.lock() == linked)
                {
                    return true;
                }
            }
            return false;
        }

        std::vector<std::shared_ptr<DynamicObject>> Frame::getLinkedFrames(void)
        {
            auto result = std::vector<std::shared_ptr<DynamicObject>>{};
            for(const auto& linkedFrame : connectedFrames)
            {
                if (const auto validFrame = linkedFrame.lock())
                {
                    result.push_back(validFrame);
                }
            }
            return result;
        }

        configmaps::ConfigMap Frame::getConfigMap() const
        {
            configmaps::ConfigMap result;

            result["name"] = getName();

            mars::utils::Vector position;
            getPosition(&position);
            result["position"] = mars::utils::vectorToConfigItem(position);

            mars::utils::Quaternion rotation;
            getRotation(&rotation);
            result["rotation"] = mars::utils::quaternionToConfigItem(rotation, true);

            configmaps::ConfigVector linkedFramesItem;
            for (const auto& linkedFrame : connectedFrames)
            {
                if (const auto validFrame = linkedFrame.lock())
                {
                    configmaps::ConfigItem frameItem;
                    frameItem = validFrame->getName();
                    linkedFramesItem.append(frameItem);
                }
            }
            result["linked frames"] = linkedFramesItem;

            mars::utils::Vector linearVelocity;
            getLinearVelocity(&linearVelocity);
            result["linear velocity"] = mars::utils::vectorToConfigItem(linearVelocity);

            mars::utils::Vector force;
            getForce(&force);
            result["force"] = mars::utils::vectorToConfigItem(force);

            mars::utils::Vector angularVelocity;
            getAngularVelocity(&angularVelocity);
            result["angular velocity"] = mars::utils::vectorToConfigItem(angularVelocity);

            mars::utils::Vector torque;
            getForce(&torque);
            result["torque"] = mars::utils::vectorToConfigItem(torque);

            mars::interfaces::sReal mass, inertia;
            getMass(&mass, &inertia);
            result["mass"] = mass;
            result["inertia"] = inertia;

            return result;
        }
        std::vector<std::string> Frame::getEditPattern(const std::string& basePath) const
        {
            return std::vector<std::string>{""};
        }
        void Frame::edit(const std::string& configPath, const std::string& value)
        {}


    } // end of namespace ode_physics
} // end of namespace mars
