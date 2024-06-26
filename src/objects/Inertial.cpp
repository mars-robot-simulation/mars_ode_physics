#include "Inertial.hpp"
#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>


namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Inertial::Inertial(std::shared_ptr<Frame> frame, ConfigMap &config) : Object(frame), config(config)
        {
            vectorFromConfigItem(config["position"], &pos);
            quaternionFromConfigItem(config["orientation"], &q);
            createMass();
        }

        Inertial::~Inertial(void)
        {
        }

        Object *Inertial::instantiate(std::shared_ptr<Frame> frame, ConfigMap &config)
        {
            return new Inertial(frame, config);
        }

        bool Inertial::createMass()
        {
            dMassSetZero(&nMass);
            nMass.mass =  (dReal)config["mass"];
            nMass.I[0] =  (dReal)config["inertia"]["i00"];
            nMass.I[1] =  (dReal)config["inertia"]["i01"];
            nMass.I[2] =  (dReal)config["inertia"]["i02"];
            nMass.I[3] =  0.0;
            nMass.I[4] =  (dReal)config["inertia"]["i10"];
            nMass.I[5] =  (dReal)config["inertia"]["i11"];
            nMass.I[6] =  (dReal)config["inertia"]["i12"];
            nMass.I[7] =  0.0;
            nMass.I[8] =  (dReal)config["inertia"]["i20"];
            nMass.I[9] =  (dReal)config["inertia"]["i21"];
            nMass.I[10] = (dReal)config["inertia"]["i22"];
            nMass.I[11] =  0.0;
            if(!dMassCheck(&nMass))
            {
                fprintf(stderr, "Mass problem:\n%s", config.toYamlString().c_str());
            }
            // dMassSetBoxTotal(&nMass, (dReal)(config["mass"]), 0.1, 0.1, 0.1);

            // TODO: add mass to frame
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
