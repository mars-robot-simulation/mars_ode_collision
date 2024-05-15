/**
 * \file CollisionHandler.hpp
 * \author Malte Langosz and Team
 * \brief "CollisionHandler" includes the methods to handle the collision between ode_collision spaces.
 *
 */

#pragma once
#include <mars_interfaces/sim/CollisionHandler.hpp>

namespace mars
{
    namespace ode_collision
    {

        class CollisionHandler : public interfaces::CollisionHandler
        {
        public:
            CollisionHandler();
            virtual ~CollisionHandler(void) override;
            virtual void getContacts(std::shared_ptr<interfaces::CollisionInterface> firstSpace,
                                     std::shared_ptr<interfaces::CollisionInterface> otherSpace,
                                     std::vector<interfaces::ContactData> &contactVector) override;
        };

    } // end of namespace ode_collision
} // end of namespace mars

