 /**
 * \file Object.hpp
 * \author Malte Langosz, Leon Danter, Muhammad Haider Khan Lodhi
 * \brief "Object" implements an Object as parent of ode objects.
 *
 */


#pragma once
#include "../CollisionSpace.hpp"
#include <mars_interfaces/contact_params.h>
#include <mars_interfaces/sim/DynamicObject.hpp>


//TODO move struct descriptions to seperate file!
// TODO: add bitmask parameter to each Object, it is currently only in box and sphere
namespace mars
{

    namespace interfaces
    {
        class GraphcisManagerInterface;
    }

    namespace ode_collision
    {

        class Object
        {
        public:
            Object(interfaces::CollisionInterface *space, std::shared_ptr<interfaces::DynamicObject> movable);
            virtual ~Object(void);

            void getPosition(utils::Vector *pos) const;
            void setPosition(const utils::Vector &pos);
            void getRotation(utils::Quaternion *q) const;
            void setRotation(const utils::Quaternion &q);

            virtual void setSize(const utils::Vector &size);
            virtual bool createGeom() = 0;
            virtual void updateTransform(void);

            bool isObjectCreated()
            {
                return objectCreated;
            }
            std::shared_ptr<interfaces::DynamicObject> getMovable() const;
            const std::string& getName() const;

            interfaces::contact_params c_params;
            double filter_depth, filter_angle, filter_radius;
            utils::Vector filter_sphere;
            // todo: find a clean solution for debug drawings
            unsigned long drawID;
            interfaces::GraphicsManagerInterface *graphics;

        protected:
            // transform is always relative to frame transformation
            // todo: do to this interlibrary connections we should switch to shared pointer
            std::shared_ptr<interfaces::DynamicObject> movable;
            utils::Vector pos;
            utils::Quaternion q;
            bool objectCreated;
            dGeomID nGeom;
            CollisionSpace *space;
            std::string name;
        };

    } // end of namespace ode_collision
} // end of namespace mars


