 /**
 * \file Object.hpp
 * \author Malte Langosz, Leon Danter, Muhammad Haider Khan Lodhi
 * \brief "Object" implements an Object as parent of ode objects.
 *
 */

#pragma once

#include "Frame.hpp"

//TODO move struct descriptions to seperate file!
namespace mars
{
    namespace ode_physics
    {

        class Frame;

        /**
         * The class that implements the NodeInterface interface.
         *
         */

        class Object
        {
        public:
            Object(std::shared_ptr<Frame> frame);
            virtual ~Object(void);

            virtual void getPosition(utils::Vector *pos) const;
            virtual const utils::Vector setPosition(const utils::Vector &pos, bool move_group = false);
            virtual void getRotation(utils::Quaternion *q) const;
            virtual const utils::Quaternion setRotation(const utils::Quaternion &q, bool move_group = false);
            virtual void getLinearVelocity(utils::Vector *vel) const;
            virtual void getAngularVelocity(utils::Vector *vel) const;
            virtual void getForce(utils::Vector *f) const;
            virtual void getTorque(utils::Vector *t) const;
            virtual const utils::Vector rotateAtPoint(const utils::Vector &rotation_point,
                                                      const utils::Quaternion &rotation, 
                                                      bool move_group);
            virtual void setLinearVelocity(const utils::Vector &velocity);
            virtual void setAngularVelocity(const utils::Vector &velocity);
            virtual void setForce(const utils::Vector &f);
            virtual void setTorque(const utils::Vector &t);
            virtual void addForce(const utils::Vector &f, const utils::Vector &p);
            virtual void addForce(const utils::Vector &f);
            virtual void addTorque(const utils::Vector &t);

            virtual void getMass(interfaces::sReal *mass, interfaces::sReal *inertia=0) const;
            dMass getMass(void) const;
            void getAbsMass(dMass *tMass) const;
            virtual bool createMass() = 0;
            const bool isObjectCreated()
            {
                return objectCreated;
            }
        protected:
            std::weak_ptr<Frame> frame;
            // transform is always relative to frame transformation
            utils::Vector pos;
            utils::Quaternion q;
            dMass nMass;
            bool objectCreated;
        };

    } // end of namespace ode_physics
} // end of namespace mars
