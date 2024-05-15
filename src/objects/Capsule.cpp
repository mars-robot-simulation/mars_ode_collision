

#include "Capsule.hpp"
#include <mars_interfaces/terrainStruct.h>

namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Capsule::Capsule(interfaces::CollisionInterface *space,std::shared_ptr<interfaces::DynamicObject> movable, ConfigMap &config) : Object(space, movable), config(config)
        {
            fprintf(stderr, "ode_collision: Capsule constructor.\n");
        }

        Capsule::~Capsule(void)
        {
        }

        Object *Capsule::instantiate(interfaces::CollisionInterface *space,std::shared_ptr<interfaces::DynamicObject> movable, ConfigMap &config)
        {
            Capsule *ca = new Capsule(space, movable, config);
            ca->createGeom();
            return ca;
        }

        bool Capsule::createGeom()
        {
            name << config["name"];
            double x = config["extend"]["x"];
            double y = config["extend"]["y"];
            // TODO: add bitmask here?

            // build the ode representation
            nGeom = dCreateCapsule(space->getSpace(), (dReal)(x),
                                   (dReal)(y));
            dGeomSetData(nGeom, this);

            objectCreated = true;
            return true;
        }

    } // end of namespace ode_collision
} // end of namespace mars
