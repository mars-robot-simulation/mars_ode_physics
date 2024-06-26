#include "CCylinder.hpp"

#include "Cylinder.hpp"
#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Cylinder::Cylinder(std::shared_ptr<Frame> frame, ConfigMap &config) : Object(frame), config(config)
        {
            vectorFromConfigItem(config["position"], &pos);
            quaternionFromConfigItem(config["orientation"], &q);
            createMass();
        }

        Cylinder::~Cylinder(void)
        {
        }

        Object *Cylinder::instantiate(std::shared_ptr<Frame> frame, ConfigMap &config)
        {
            return new Cylinder(frame, config);
        }

        bool Cylinder::createMass()
        {
            const dReal radius = (dReal)config["extend"]["x"];
            const dReal length = (dReal)config["extend"]["y"];
            const dReal density = (dReal)(config["density"]);
            const dReal mass = (dReal)(config["mass"]);

            dMassSetCylinder(&nMass, density, 3, radius, length);
            dMassSetCylinderTotal(&nMass, mass, 3, radius, length);

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
