

#include "Capsule.hpp"
#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Capsule::Capsule(std::shared_ptr<Frame> frame, ConfigMap &config) : Object(frame), config(config)
        {
            vectorFromConfigItem(config["position"], &pos);
            quaternionFromConfigItem(config["orientation"], &q);
            createMass();
        }

        Capsule::~Capsule(void)
        {
        }

        Object *Capsule::instantiate(std::shared_ptr<Frame> frame, ConfigMap &config)
        {
            return new Capsule(frame, config);
        }

        bool Capsule::createMass()
        {
            const dReal radius = (dReal)config["extend"]["x"];
            const dReal length = (dReal)config["extend"]["y"];
            const dReal density = (dReal)(config["density"]);
            const dReal mass = (dReal)(config["mass"]);

            dMassSetCapsule(&nMass, density, 3, radius, length);
            dMassSetCapsuleTotal(&nMass, mass, 3, radius, length);

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
