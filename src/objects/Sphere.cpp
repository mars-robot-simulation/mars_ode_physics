#include "Sphere.hpp"

namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Sphere::Sphere(std::shared_ptr<Frame> frame, ConfigMap &config) : Object(frame), config(config)
        {
            vectorFromConfigItem(config["position"], &pos);
            quaternionFromConfigItem(config["orientation"], &q);
            createMass();
        }

        Sphere::~Sphere(void)
        {
        }

        Object *Sphere::instantiate(std::shared_ptr<Frame> frame, ConfigMap &config)
        {
            return new Sphere(frame, config);
        }

        /**
         * The method creates an ode sphere representation of the given node.
         *
         */
        bool Sphere::createMass()
        {
            const dReal radius = (dReal)config["extend"]["x"];
            const dReal density = (dReal)(config["density"]);
            const dReal mass = (dReal)(config["mass"]);

            dMassSetSphere(&nMass, density, radius);
            dMassSetSphereTotal(&nMass, mass, radius);

            // TODO: Add mass to frame
            if (auto validFrame = frame.lock())
            {
                validFrame->addObject(this);
            }

            // TODO: Handle invalid frame?
            objectCreated = true;
            return true;
        } 

    } // end of namespace ode_physics
} // end of namespace mars
