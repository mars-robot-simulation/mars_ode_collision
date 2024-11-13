
#include "Cylinder.hpp"
#include <mars_interfaces/terrainStruct.h>

namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

    

        Cylinder::Cylinder(interfaces::CollisionInterface *space,std::shared_ptr<interfaces::DynamicObject> movable, ConfigMap &config) : Object(space, movable, config)
        {
            LOG_INFO("ode_collision: Cylinder constructor.\n");
        }

        Cylinder::~Cylinder(void)
        {
        }

        Object *Cylinder::instantiate(interfaces::CollisionInterface *space,std::shared_ptr<interfaces::DynamicObject> movable, ConfigMap &config)
        {
            Cylinder *c = new Cylinder(space, movable, config);
            c->createGeom();
            return c;
        }

        bool Cylinder::createGeom()
        {
            name << config["name"];
            double x = config["extend"]["x"];
            double y = config["extend"]["y"];
            // build the ode representation
            nGeom = dCreateCylinder(space->getSpace(), (dReal)(x),
                                    (dReal)(y));

            dGeomSetData(nGeom, this);

            objectCreated = true;
            return true;
        }

    }
}
