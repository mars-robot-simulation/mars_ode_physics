//TODO cleanup includes
#include "Frame.hpp"
#include "Object.hpp"

#include <mars_interfaces/Logging.hpp>
#include <mars_utils/MutexLocker.h>
#include <mars_utils/mathUtils.h>
#include <cmath>
#include <set>

#include <ode/odemath.h>

#include <iostream>


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;

        /**
         * \brief Creates a empty node objekt.
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
        Object::Object(std::shared_ptr<Frame> frame) : frame(frame), objectCreated(false),
                                       pos(0.0, 0.0, 0.0),
                                       q(1.0, 0.0, 0.0, 0.0)

        {
            dMassSetZero(&nMass);
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
        Object::~Object(void)
        {
            //std::vector<sensor_list_element>::iterator iter;
            //MutexLocker locker(&(theWorld->iMutex));

            // todo: remove object from frame?
        }

        void Object::getPosition(Vector* pos) const
        {
            // todo: position is defined by frame position
            // local position and rotation should store offsets
            //MutexLocker locker(&(theWorld->iMutex));
            *pos = this->pos;
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
        const Vector Object::setPosition(const Vector &pos, bool move_group)
        {
            // todo: update the position of the frame or center of mass of this object in the frame
            this->pos = pos;
            return Vector();
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
        void Object::getRotation(Quaternion* q) const
        {
            // todo: see above
            *q = this->q;
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
        void Object::getLinearVelocity(Vector* vel) const
        {
            // todo
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
        void Object::getAngularVelocity(Vector* vel) const
        {
            // todo
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
        void Object::getForce(Vector* f) const
        {
            // todo
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
        void Object::getTorque(Vector *t) const
        {
            // todo
        }


        /**
         * \brief This method sets the rotation of the physically node.
         *
         * I don't if and how to use this function yet. ^-^
         * If we need it, the pre and post conditions are like them in the set
         * position method.
         */
        const Quaternion Object::setRotation(const Quaternion &q, bool move_group)
        {
            // todo
            this->q = q;
            return Quaternion();
        }



        /**
         * \brief executes an rotation at a given point and returns the
         * new position of the node
         *
         * pre:
         *
         * post:
         */
        const Vector Object::rotateAtPoint(const Vector &rotation_point,
                                           const Quaternion &rotation,
                                           bool move_group)
        {
            // todo
            return Vector();
        }

        /**
         * \brief returns the ode mass object
         *
         *
         * pre:
         *
         * post:
         */
        dMass Object::getMass(void) const
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
        void Object::setLinearVelocity(const Vector &velocity)
        {
            // todo
            frame->setLinearVelocity(velocity);
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
        void Object::setAngularVelocity(const Vector &velocity)
        {
            // todo
            //MutexLocker locker(&(theWorld->iMutex));
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
        void Object::setForce(const Vector &f)
        {
            // todo
            //MutexLocker locker(&(theWorld->iMutex));
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
        void Object::setTorque(const Vector &t)
        {
            // todo
            //MutexLocker locker(&(theWorld->iMutex));
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
        void Object::addForce(const Vector &f, const Vector &p)
        {
            // todo
            //MutexLocker locker(&(theWorld->iMutex));
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
        void Object::addForce(const Vector &f)
        {
            // todo
            //MutexLocker locker(&(theWorld->iMutex));
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
        void Object::addTorque(const Vector &t)
        {
            // todo
            //MutexLocker locker(&(theWorld->iMutex));
        }


        void Object::getAbsMass(dMass *tMass) const
        {
            // todo
            // no lock because physics internal functions get locked elsewhere
            // const dReal *pos = dGeomGetPosition(nGeom);
            // const dReal *rot = dGeomGetRotation(nGeom);

            // *tMass = nMass;
            // dMassRotate(tMass, rot);
            // dMassTranslate(tMass, pos[0], pos[1], pos[2]);
        }

        void Object::getMass(sReal *mass, sReal *inertia) const
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


    } // end of namespace ode_physics
} // end of namespace mars
