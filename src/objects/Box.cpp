#include "Box.hpp"

namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Box::Box(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap &config) : Object(space, movable, config)
        {
            LOG_INFO("ode_collision: Box constructor.\n");
        }

        Box::~Box(void)
        {
        }

        Object* Box::instantiate(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap &config)
        {
            Box *b = new Box(space, movable, config);
            b->createGeom();
            return b;
        }

        bool Box::createGeom()
        {
            name << config["name"];
            double x = config["extend"]["x"];
            double y = config["extend"]["y"];
            double z = config["extend"]["z"];
            if(config.hasKey("bitmask"))
            {
                c_params.coll_bitmask = config["bitmask"];
            }
            // build the ode representation
            nGeom = dCreateBox(space->getSpace(), (dReal)(x),
                               (dReal)(y), (dReal)(z));
            dGeomSetData(nGeom, this);

            objectCreated = true;
            return true;
        }

    } // end of namespace ode_collision
} // end of namespace mars
