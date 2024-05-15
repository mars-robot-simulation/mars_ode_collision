/**
 * \file CollisionHandler.cpp
 * \author Malte Langosz and team
 * \brief "CollisionHandler" includes the methods to handle collisions of ode_collision spaces.
 *
 */

#include "CollisionHandler.hpp"
#include "CollisionSpace.hpp"
#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <mars_interfaces/sim/CollisionInterface.hpp>


namespace mars
{
    namespace ode_collision
    {
        CollisionHandler::CollisionHandler()
        {
        }

        CollisionHandler::~CollisionHandler(void)
        {
        }

        void CollisionHandler::getContacts(std::shared_ptr<interfaces::CollisionInterface> firstSpace,
                                           std::shared_ptr<interfaces::CollisionInterface> secondSpace,
                                           std::vector<interfaces::ContactData> &contactVector)
        {
            CollisionSpace *firstSpace_ = dynamic_cast<CollisionSpace*>(firstSpace.get());
            firstSpace_->getContacts(secondSpace, contactVector);
        }

    } // end of namespace ode_collision
} // end of namespace mars
