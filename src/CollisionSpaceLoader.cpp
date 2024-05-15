/**
 * \file CollisionSpaceLoader.cpp
 * \author Malte Langosz
 * \brief "CollisionSpaceLoader" factory class to generate new physics instances.
 *
 */

#include "CollisionSpaceLoader.hpp"
#include "objects/ObjectFactory.hpp"
#include "objects/Box.hpp"
#include "objects/Plane.hpp"
#include "objects/Sphere.hpp"
#include "objects/Heightfield.hpp"
#include "objects/Cylinder.hpp"
#include "objects/Capsule.hpp"
#include "objects/Mesh.hpp"

namespace mars
{
    namespace ode_collision
    {

        CollisionSpaceLoader::CollisionSpaceLoader(lib_manager::LibManager *theManager) : lib_manager::LibInterface(theManager)
        {
            ObjectFactory::Instance().addObjectType("box", &Box::instantiate);
            ObjectFactory::Instance().addObjectType("plane", &Plane::instantiate);
            ObjectFactory::Instance().addObjectType("sphere", &Sphere::instantiate);
            ObjectFactory::Instance().addObjectType("heightfield", &Heightfield::instantiate);
            ObjectFactory::Instance().addObjectType("mesh", &Mesh::instantiate);
            ObjectFactory::Instance().addObjectType("cylinder", &Cylinder::instantiate);
            ObjectFactory::Instance().addObjectType("capsule", &Capsule::instantiate);        
        }

        CollisionSpaceLoader::~CollisionSpaceLoader(void)
        {

        }

        std::shared_ptr<interfaces::CollisionInterface> CollisionSpaceLoader::createCollisionSpace(interfaces::ControlCenter *control)
        {
            std::shared_ptr<CollisionSpace> collisionSpace = std::make_shared<CollisionSpace>(control);
            return std::static_pointer_cast<interfaces::CollisionInterface>(collisionSpace);
        }

    } // end of namespace ode_collision
} // end of namespace mars

DESTROY_LIB(mars::ode_collision::CollisionSpaceLoader)
CREATE_LIB(mars::ode_collision::CollisionSpaceLoader)
