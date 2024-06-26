#include "Box.hpp"
#include <mars_utils/misc.h>
#include <mars_utils/mathUtils.h>


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Box::Box(std::shared_ptr<Frame> frame, ConfigMap &config) : Object(frame), config(config)
        {
            vectorFromConfigItem(config["position"], &pos);
            quaternionFromConfigItem(config["orientation"], &q);
            createMass();
        }

        Box::~Box(void)
        {
        }

        Object *Box::instantiate(std::shared_ptr<Frame> frame, ConfigMap &config)
        {
            return new Box(frame, config);
        }

        bool Box::createMass()
        {
            const dReal x = (dReal)config["extend"]["x"];
            const dReal y = (dReal)config["extend"]["y"];
            const dReal z = (dReal)config["extend"]["z"];
            const dReal density = (dReal)(config["density"]);
            const dReal mass = (dReal)(config["mass"]);

            dMassSetBox(&nMass, density, x, y, z);
            dMassSetBoxTotal(&nMass, mass,  x, y, z);

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
