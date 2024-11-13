#include "Plane.hpp"

namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Plane::Plane(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap &config) : Object(space, movable, config)
        {
            LOG_INFO("ode_collision: Plane constructor.\n");
        }

        Plane::~Plane(void)
        {
        }

        Object* Plane::instantiate(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap &config)
        {
            Plane* s = new Plane(space, movable, config);
            s->createGeom();
            return s;
        }

        /**
         * The method creates an ode plane representation of the given node.
         *
         */
        bool Plane::createGeom()
        {
            double zPosition;
            name << config["name"];
            zPosition = config["position"]["z"];
            // build the ode representation
            nGeom = dCreatePlane(space->getSpace(), 0, 0, 1, (dReal)zPosition);
            dGeomSetData(nGeom, this);

            objectCreated = true;
            return true;
        }

        void Plane::updateTransform(void)
        {
            // todo: update z-position of plane
        }

    } // end of namespace ode_collision
} // end of namespace mars
