#include "ODEInertial.hpp"

namespace mars
{
    namespace ode_physics
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        ODEInertial::ODEInertial(ODEFrame *frame, ConfigMap &config) : ODEObject(frame), config(config)
        {
        }

        ODEInertial::~ODEInertial(void)
        {
        }

        ODEObject* ODEInertial::instanciate(ODEFrame *frame, ConfigMap &config)
        {
            return new ODEInertial(frame, config);
        }

        bool ODEInertial::createODEMass(){
            if(!config.hasKey("inertia") or config["inertia"].size() < 9 or !config.hasKey("mass") or (double)config["mass"] < 0.0)
            {
                // todo: be more informative
                LOG_ERROR("Cannot create ODEInertial since config data is not matching\n");
                return false;
            }
            dMassSetZero(&nMass);
            nMass.mass =  (dReal)config["mass"];
            nMass.I[0] =  (dReal)config["inertia"]["i00"];
            nMass.I[1] =  (dReal)config["inertia"]["i01"];
            nMass.I[2] =  (dReal)config["inertia"]["i02"];
            nMass.I[4] =  (dReal)config["inertia"]["i10"];
            nMass.I[5] =  (dReal)config["inertia"]["i11"];
            nMass.I[6] =  (dReal)config["inertia"]["i12"];
            nMass.I[8] =  (dReal)config["inertia"]["i20"];
            nMass.I[9] =  (dReal)config["inertia"]["i21"];
            nMass.I[10] = (dReal)config["inertia"]["i22"];

            // todo: add mass to frame
            frame->addObject(this);
            objectCreated = true;
            return true;
        }

    } // end of namespace ode_physics
} // end of namespace mars
