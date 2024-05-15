//TODO cleanup includes
#include "Object.hpp"

#include <mars_interfaces/Logging.hpp>
#include <mars_utils/MutexLocker.h>
#include <mars_utils/mathUtils.h>

#include <mars_interfaces/Logging.hpp>

#include <iostream>


namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;

        /**
         * \brief Creates a empty node object.
         *
         * \pre
         *     - the pointer to the physics Interface should be correct.
         *       This implementation can be a bad trap. The class that implements the
         *       physics interface, have to inherit from the interface at first,
         *       otherwise this implementation will cause bad error while pointing
         *       an incorrect adresses.
         *
         * \post
         *     - the class should have saved the pointer to the physics implementation
         *     - the body and geom should be initialized to 0
         */
        Object::Object(CollisionInterface *space, std::shared_ptr<DynamicObject> movable) : movable(movable), objectCreated(false),
                                                                                            pos(0.0, 0.0, 0.0),
                                                                                            q(1.0, 0.0, 0.0, 0.0),
                                                                                            filter_depth(-1.0),
                                                                                            filter_angle(-1.0),
                                                                                            filter_radius(-1.0),
                                                                                            filter_sphere(0.0, 0.0, 0.0)
        {
            this->space = dynamic_cast<CollisionSpace*>(space);
            graphics = NULL;
        }

        /**
         * \brief Destroys the node in the physical world.
         *
         * pre:
         *     - theWorld is the correct world object
         *
         * post:
         *     - all physical representation of the node should be cleared
         *
         * are the geom and the body realy all thing to take care of?
         */
        Object::~Object(void)
        {
            //std::vector<sensor_list_element>::iterator iter;
            //MutexLocker locker(&(theWorld->iMutex));

            // todo: remove object from frame?
        }

        void Object::getPosition(Vector* pos) const
        {
            // todo: position is defined by frame position
            // local position and rotation should store offsets
            //MutexLocker locker(&(theWorld->iMutex));
            const dReal *dPos = dGeomGetPosition(nGeom);
            pos->x() = dPos[0];
            pos->y() = dPos[1];
            pos->z() = dPos[2];
        }

        /**
         * \brief The method sets the position of the physical node model to the
         * position of the param.  If move_group is set, all nodes of a composite
         * group will be moved, otherwise only this node will be moved.  A vector
         * from the old position to the new will be returned.
         *
         * I don't know if we should use this function in a way like it is
         * implemented now. The pre and post conditions could loke like this:
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the pos param should point to a correct position struct
         *
         * post:
         *     - if there is a physically representation and the node is movable
         *       set the position of the corresponding body to the given parameter
         *     - otherwise, we have to do nothing
         */
        void Object::setPosition(const Vector &pos)
        {
            // todo: update the position of the frame or center of mass of this object in the frame
            this->pos = pos;
        }

        /**
         * \brief The method copies the Quaternion of the physically node at the
         * adress of the Quaternion pointer q.
         *
         * pre:
         *     - there should be a physical representation of the node
         *     - the node should be movable
         *     - the q param should point to a correct Quaternion struct
         *
         * post:
         *     - if there is a physical representation and the node is movable
         *       the Quaternion struct should be filled with the physical rotation
         *       of the node
         *     - otherwise a standard return of zero rotation should be set
         */
        // todo: add getAbsoluteRotation ...
        void Object::getRotation(Quaternion* q) const
        {
            // todo: see above
            dQuaternion dQ;
            dGeomGetQuaternion(nGeom, dQ);
            q->w() = dQ[0];
            q->x() = dQ[1];
            q->y() = dQ[2];
            q->z() = dQ[3];
        }

        /**
         * \brief This method sets the rotation of the physically node.
         *
         * I don't if and how to use this function yet. ^-^
         * If we need it, the pre and post conditions are like them in the set
         * position method.
         */
        void Object::setRotation(const Quaternion &q)
        {
            // todo
            this->q = q;
        }

        // todo: change name to updateAbsTransform
        void Object::updateTransform(void)
        {
            // local transform is relative to parent frame
            //fprintf(stderr, "update collision of ");
            if(movable)
            {
                //fprintf(stderr, "  %s\n", movable->getName().c_str());
                Vector framePos;
                Quaternion frameQ;
                movable->getPosition(&framePos);
                movable->getRotation(&frameQ);

                framePos += frameQ*pos;
                // todo: dGeom set position
                dGeomSetPosition(nGeom, (dReal)framePos.x(),
                                 (dReal)framePos.y(), (dReal)framePos.z());
                //fprintf(stderr, " %g %g %g - ", framePos.x(), framePos.y(), framePos.z());
                frameQ = frameQ*q;
                dQuaternion dQ = {frameQ.w(), frameQ.x(), frameQ.y(), frameQ.z()};
                dGeomSetQuaternion(nGeom, dQ);
                //fprintf(stderr, " %g %g %g %g\n", frameQ.w(), frameQ.x(), frameQ.y(), frameQ.z());
            } else
            {
                //fprintf(stderr, "static\n");
                dGeomSetPosition(nGeom, (dReal)pos.x(),
                                 (dReal)pos.y(), (dReal)pos.z());
                dQuaternion dQ = {q.w(), q.x(), q.y(), q.z()};
                dGeomSetQuaternion(nGeom, dQ);
            }
        }

        std::shared_ptr<DynamicObject> Object::getMovable() const
        {
            return movable;
        }

        const std::string& Object::getName() const
        {
            return name;
        }

        void Object::setSize(const Vector &size)
        {
            LOG_WARN("Set size not implemented for object %s", name.c_str());
        }

    } // end of namespace ode_collision
} // end of namespace mars
