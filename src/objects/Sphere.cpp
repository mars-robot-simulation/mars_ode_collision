#include "Sphere.hpp"

namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Sphere::Sphere(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap &config) : Object(space, movable, config)
        {
            LOG_INFO("ode_collision: Sphere constructor.\n");
        }

        Sphere::~Sphere(void)
        {
        }

        Object* Sphere::instantiate(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap &config)
        {
            Sphere* s = new Sphere(space, movable, config);
            s->createGeom();
            return s;
        }

        /**
         * The method creates an ode shpere representation of the given node.
         *
         */
        bool Sphere::createGeom()
        {
            double radius;
            name << config["name"];
            if(config.hasKey("bitmask"))
            {
                c_params.coll_bitmask = config["bitmask"];
            }
            radius = config["extend"]["x"];
            // build the ode representation
            nGeom = dCreateSphere(space->getSpace(), (dReal)radius);
            dGeomSetData(nGeom, this);

            objectCreated = true;
            return true;
        }

    } // end of namespace ode_collision
} // end of namespace mars
