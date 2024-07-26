/**
 * \file CollisionSpace.cpp
 * \author Malte Langosz and team
 * \brief "CollisionSpace" includes the methods to handle collision spaces.
 *
 */

#include "CollisionSpace.hpp"
#include "objects/Object.hpp"
#include "objects/ObjectFactory.hpp"

#include <mars_utils/MutexLocker.h>
#include <mars_interfaces/Logging.hpp>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
#include <mars_utils/misc.h>
#include <cfg_manager/CFGManagerInterface.h>

#include <configmaps/ConfigSchema.hpp>

#define EPSILON 1e-10


namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;

        /**
         *  \brief The constructor for the collision space.
         *
         *  pre:
         *      - none
         *
         *  post:
         *      - all private variables should be initialized correct
         *        should correct be spezified?
         *      - world, space, contactgroup and world_init to false (0)
         */
        // TODO: we only need a physics not the control center?
        CollisionSpace::CollisionSpace(interfaces::ControlCenter *control) : 
            interfaces::CollisionInterface(), control{control}
        {
            ground_friction = 20;
            ground_cfm = 0.00000001;
            ground_erp = 0.1;
            space = 0;
            space_init = 0;
            num_contacts = 0;
            create_contacts = 1;
            log_contacts = 0;
            registerSchemaValidators();
        }

        /**
         * \brief Close ODE environment
         *
         * pre:
         *     - none
         *
         * post:
         *     - everthing that was created should be destroyed
         *
         */
        CollisionSpace::~CollisionSpace(void)
        {
            MutexLocker locker(&iMutex);
            for(auto namedObject : objects)
            {
                delete namedObject.second;
            }
            dCloseODE();
        }

        /**
         *  \brief This function initializes the ode world.
         *
         * pre:
         *     - world_init = false
         *
         * post:
         *     - world, space and contactgroup should be created
         *     - the ODE world parameters should be set
         *     - at the end world_init have to become true
         */
        void CollisionSpace::initSpace(void)
        {
            MutexLocker locker(&iMutex);

            dInitODE();
            // if world_init = true debug something
            if(!space_init)
            {
                // LOG_DEBUG("init physics world");
                space = dHashSpaceCreate(0);

                space_init = 1;
            }
        }

        /**
         * \brief This functions destroys the ode world.
         *
         * pre:
         *     - world_init = true
         *
         * post:
         *     - world, space and contactgroup have to be destroyed here
         *     - afte that, world_init have to become false
         */
        void CollisionSpace::freeSpace(void)
        {
            reset();

            MutexLocker locker(&iMutex);
            if(space_init)
            {
                // LOG_DEBUG("free physics world");
                dSpaceDestroy(space);
                space_init = 0;
            }
            // else debug something
        }

        /**
         * \brief Returns if a world exists.
         *
         * pre:
         *     - none
         *
         * post:
         *     - return state of world_init
         */
        bool CollisionSpace::existsSpace(void) const
        {
            return space_init;
        }

       /**
         * \brief Register schema validators for objects and joints of this collision space
         * \note config will be validated by the corresponding schema before the creation of any Object by CollisionSpace::createObject
         */
        void CollisionSpace::registerSchemaValidators()
        {
            const MutexLocker locker{&iMutex};

            // Register schema validators for objects of this collision space
            for(const auto &[obj_name, _] : ObjectFactory::Instance().getAvailableObjects())
            {
                std::string obj_schema_path = mars::utils::pathJoin(SCHEMA_PATH, "objects/" + obj_name + "_schema.yaml");
                if(!mars::utils::pathExists(obj_schema_path))
                {
                    LOG_ERROR("Missing schema file " + obj_schema_path + " for " + obj_name);
                    continue;
                }
                configmaps::ConfigSchema schema(configmaps::ConfigMap::fromYamlFile(obj_schema_path));
                objects_schema[obj_name] = schema;
            }
        }

        /**
         * \brief This function handles the calculation of a step in the world.
         *
         * pre:
         *     - space_init = true
         *
         * post:
         *     - handled the collisions
         *     - the contactgroup should be empty
         */
        void CollisionSpace::generateContacts()
        {
            const MutexLocker locker{&iMutex};
            // std::vector<dJointFeedback*>::iterator iter;

            // if world_init = false or step_size <= 0 debug something
            if(space_init > 0)
            {
                /// first check for collisions
                num_contacts = log_contacts = 0;
                contactVector.clear();
                dSpaceCollide(space, this, &CollisionSpace::callbackForward);
            }
        }

        /**
         * \brief In this function the collision handling from ode is performed.
         *
         * pre:
         *     - world_init = true
         *     - o1 and o2 are regular geoms
         *
         * post:
         *     - if o1 or o2 was a Space, called SpaceCollide and exit
         *     - otherwise tested if the geoms collide and created a contact
         *       joint if so.
         *
         * A lot of the code is uncommented in this function. This
         * code maybe used later to handle sensors or other special cases
         * in the simulation.
         */
        void CollisionSpace::nearCallback(dGeomID o1, dGeomID o2)
        {
            int i;
            int numc;
            // up to MAX_CONTACTS contact per Box-box
            // dContact contact[MAX_CONTACTS];
            dVector3 v1, v;
            // dMatrix3 R;
            dReal dot;

            if(dGeomIsSpace(o1) || dGeomIsSpace(o2))
            {
                /// test if a space is colliding with something
                dSpaceCollide2(o1, o2, this, &CollisionSpace::callbackForward);
                return;
            }

            /// exit without doing anything if the two bodies are connected by a joint
            // dBodyID b1=dGeomGetBody(o1);
            // dBodyID b2=dGeomGetBody(o2);

            const auto* const object1 = reinterpret_cast<Object*>(dGeomGetData(o1));
            const auto* const object2 = reinterpret_cast<Object*>(dGeomGetData(o2));
            if(!(object1->c_params.coll_bitmask & object2->c_params.coll_bitmask))
            {
                return;
            }

            auto d1 = object1->getMovable();
            auto d2 = object2->getMovable();
            // fprintf(stderr, "check collision of: %s / %s\n", object1->getName().c_str(), object2->getName().c_str());
            if(d1 == d2)
            {
                // fprintf(stderr, "\t\treturn since movables are the same\n");
                return;
            }
            if(d1 && d2 && d1->isLinkedFrame(d2))
            {
                return;
            }
            // TODO: add method to check if these objects can collide
            // for example have a std::map as blacklist for collisions in the objects
            // Objects cannot collide if they are connected, or if they are in same group id's or
            // for what ever reason

            int maxNumContacts = 0;
            // TODO: how to handle contact parameters
            if(object1->c_params.max_num_contacts <
                object2->c_params.max_num_contacts)
            {
                maxNumContacts = object1->c_params.max_num_contacts;
            } else
            {
                maxNumContacts = object2->c_params.max_num_contacts;
            }
            // fprintf(stderr, "\tmax_num_contacts: %d\n", maxNumContacts);
            //  todo: for testing we override the max num contacts with one
            // maxNumContacts = 1;
            // TODO: Replaceable with vector?
            auto* const contact = new dContact[maxNumContacts];

            // TODO: Replaceable with std::max?
            double filter_depth = -1.0;
            if(object1->filter_depth > filter_depth)
            {
                filter_depth = object1->filter_depth;
            }
            if(object2->filter_depth > filter_depth)
            {
                filter_depth = object2->filter_depth;
            }
            // TODO: Replaceable with std::max?
            double filter_angle = 0.5;
            if(object1->filter_angle > 0.0)
            {
                filter_angle = object1->filter_angle;
            }
            if(object2->filter_angle > 0.0 and object2->filter_angle > object1->filter_angle)
            {
                filter_angle = object2->filter_angle;
            }

            // TODO: Replaceable with std::max and conditional for sphere?
            double filter_radius = -1.0;
            Vector filter_sphere(0.0, 0.0, 0.0);

            if(object1->filter_radius > filter_radius)
            {
                filter_radius = object1->filter_radius;
                filter_sphere = object1->filter_sphere;
            }
            if(object2->filter_radius > filter_radius)
            {
                filter_radius = object2->filter_radius;
                filter_sphere = object2->filter_sphere;
            }

            // frist we set the softness values:
            contact[0].surface.mode = dContactSoftERP | dContactSoftCFM;
            contact[0].surface.soft_cfm = (object1->c_params.cfm +
                                           object2->c_params.cfm) /
                                          2;
            contact[0].surface.soft_erp = (object1->c_params.erp +
                                           object2->c_params.erp) /
                                          2;
            // then check if one of the geoms want to use the pyramid approximation
            if(object1->c_params.approx_pyramid ||
                object2->c_params.approx_pyramid)
                contact[0].surface.mode |= dContactApprox1;

            // Then check the friction for both directions
            contact[0].surface.mu = (object1->c_params.friction1 +
                                     object2->c_params.friction1) /
                                    2;
            contact[0].surface.mu2 = (object1->c_params.friction2 +
                                      object2->c_params.friction2) /
                                     2;

            if(contact[0].surface.mu != contact[0].surface.mu2)
            {
                contact[0].surface.mode |= dContactMu2;
            }

            if(object1->c_params.rolling_friction > EPSILON ||
                object2->c_params.rolling_friction > EPSILON)
            {
                contact[0].surface.mode |= dContactRolling;
                contact[0].surface.rho = object1->c_params.rolling_friction + object2->c_params.rolling_friction;
                // fprintf(stderr, "set rolling friction to: %g\n", contact[0].surface.rho);
                if(object1->c_params.rolling_friction2 > EPSILON ||
                    object2->c_params.rolling_friction2 > EPSILON)
                {
                    contact[0].surface.rho2 = object1->c_params.rolling_friction2 + object2->c_params.rolling_friction2;
                } else
                {
                    contact[0].surface.rho2 = object1->c_params.rolling_friction + object2->c_params.rolling_friction;
                }
                if(object1->c_params.spinning_friction > EPSILON ||
                    object2->c_params.spinning_friction > EPSILON)
                {
                    contact[0].surface.rhoN = object1->c_params.spinning_friction + object2->c_params.spinning_friction;
                } else
                {
                    contact[0].surface.rhoN = 0.0;
                }
            }

            // check if we have to calculate friction direction1
            if(object1->c_params.friction_direction1 ||
                object2->c_params.friction_direction1)
            {
                // here the calculation becomes more complicated
                // maybe we should make some restrictions
                // so -> we only use friction motion in friction direction 1
                // the friction motion is only set if a local vector for friction
                // direction 1 is given
                // the steps for the calculation:
                // 1. rotate the local vectors to global coordinates
                // 2. scale the vectors to the length of the motion if given
                // 3. vector 3 =  vector 1 - vector 2
                // 4. get the length of vector 3
                // 5. set vector 3 as friction direction 1
                // 6. set motion 1 to the length
                contact[0].surface.mode |= dContactFDir1;
                if(!object2->c_params.friction_direction1)
                {
                    // get the orientation of the geom
                    // dGeomGetQuaternion(o1, v);
                    // dRfromQ(R, v);
                    // copy the friction direction
                    v1[0] = object1->c_params.friction_direction1->x();
                    v1[1] = object1->c_params.friction_direction1->y();
                    v1[2] = object1->c_params.friction_direction1->z();
                    // translate the friction direction to global coordinates
                    // and set friction direction for contact
                    // dMULTIPLY0_331(contact[0].fdir1, R, v1);
                    contact[0].fdir1[0] = v1[0];
                    contact[0].fdir1[1] = v1[1];
                    contact[0].fdir1[2] = v1[2];
                    if(object1->c_params.motion1)
                    {
                        contact[0].surface.mode |= dContactMotion1;
                        contact[0].surface.motion1 = object1->c_params.motion1;
                    }
                } else if(!object1->c_params.friction_direction1)
                {
                    // get the orientation of the geom
                    // dGeomGetQuaternion(o2, v);
                    // dRfromQ(R, v);
                    // copy the friction direction
                    v1[0] = object2->c_params.friction_direction1->x();
                    v1[1] = object2->c_params.friction_direction1->y();
                    v1[2] = object2->c_params.friction_direction1->z();
                    // translate the friction direction to global coordinates
                    // and set friction direction for contact
                    // dMULTIPLY0_331(contact[0].fdir1, R, v1);
                    contact[0].fdir1[0] = v1[0];
                    contact[0].fdir1[1] = v1[1];
                    contact[0].fdir1[2] = v1[2];
                    if(object2->c_params.motion1)
                    {
                        contact[0].surface.mode |= dContactMotion1;
                        contact[0].surface.motion1 = object2->c_params.motion1;
                    }
                } else
                {
                    // the calculation steps as mentioned above
                    fprintf(stderr, "the calculation for friction directen set for both nodes is not done yet.\n");
                }
            }

            // then check for fds
            if(object1->c_params.fds1 || object2->c_params.fds1)
            {
                contact[0].surface.mode |= dContactSlip1;
                contact[0].surface.slip1 = (object1->c_params.fds1 +
                                            object2->c_params.fds1);
            }
            if(object1->c_params.fds2 || object2->c_params.fds2)
            {
                contact[0].surface.mode |= dContactSlip2;
                contact[0].surface.slip2 = (object1->c_params.fds2 +
                                            object2->c_params.fds2);
            }
            if(object1->c_params.bounce || object2->c_params.bounce)
            {
                contact[0].surface.mode |= dContactBounce;
                contact[0].surface.bounce = (object1->c_params.bounce +
                                             object2->c_params.bounce);
                if(object1->c_params.bounce_vel > object2->c_params.bounce_vel)
                    contact[0].surface.bounce_vel = object1->c_params.bounce_vel;
                else
                    contact[0].surface.bounce_vel = object2->c_params.bounce_vel;
            }

            for(i=1; i<maxNumContacts; i++)
            {
                contact[i] = contact[0];
            }

            // fprintf(stderr, "\tcall dCollide\n");

            numc = dCollide(o1, o2, maxNumContacts, &contact[0].geom, sizeof(dContact));
            if(numc)
            {
                Vector contact_point;

                // TODO: add depth handling here too
                bool have_contact = false;
                for(i=0; i<numc; i++)
                {
                    // filter_depth is used to filter heightmaps contact under the surface
                    if(filter_depth > 0.0)
                    {
                        if(contact[i].geom.normal[2] < 0.5 or filter_depth < contact[i].geom.depth)
                        {
                            continue;
                        }
                    }
                    if(filter_radius > 0.0)
                    {
                        Vector v;
                        v.x() = contact[i].geom.pos[0];
                        v.y() = contact[i].geom.pos[1];
                        v.z() = 0.0; // contact[i].geom.pos[2];
                        v -= filter_sphere;
                        if(v.norm() <= filter_radius)
                        {
                            continue;
                        }
                    }
                    have_contact = true;
                    break;
                }
                if(have_contact)
                {
                    num_contacts++;
                }
                if(create_contacts)
                {
                    for(i=0; i<numc; i++)
                    {
                        // filter_depth is used to filter heightmaps contact under the surface
                        if(filter_depth > 0.0)
                        {
                            if(contact[i].geom.normal[2] < 0.5 or filter_depth < contact[i].geom.depth)
                            {
                                // fprintf(stderr, "\t\tfilter contact\n");
                                continue;
                            }
                        }
                        if(filter_radius > 0.0)
                        {
                            Vector v;
                            v.x() = contact[i].geom.pos[0];
                            v.y() = contact[i].geom.pos[1];
                            v.z() = 0.0; // contact[i].geom.pos[2];
                            v -= filter_sphere;
                            if(v.norm() <= filter_radius)
                            {
                                // fprintf(stderr, "\t\tfilter contact\n");
                                continue;
                            }
                        }
                        if(object1->c_params.friction_direction1 ||
                            object2->c_params.friction_direction1)
                        {
                            v[0] = contact[i].geom.normal[0];
                            v[1] = contact[i].geom.normal[1];
                            v[2] = contact[i].geom.normal[2];
                            dot = dDOT(v, contact[i].fdir1);
                            dOPEC(v, *=, dot);
                            contact[i].fdir1[0] -= v[0];
                            contact[i].fdir1[1] -= v[1];
                            contact[i].fdir1[2] -= v[2];
                            dNormalize3(contact[0].fdir1);
                        }
                        contact[i].geom.depth += (object1->c_params.depth_correction +
                                                  object2->c_params.depth_correction);

                        if(contact[i].geom.depth < 0.0)
                            contact[i].geom.depth = 0.0;
                        // transfer data from contact[i] to ContactData and store it in the contact list
                        // also store frame1 and frame2 in ContactData
                        ContactData cd;
                        cd.pos.x() = contact[i].geom.pos[0];
                        cd.pos.y() = contact[i].geom.pos[1];
                        cd.pos.z() = contact[i].geom.pos[2];
                        cd.depth = contact[i].geom.depth;
                        cd.normal.x() = contact[i].geom.normal[0];
                        cd.normal.y() = contact[i].geom.normal[1];
                        cd.normal.z() = contact[i].geom.normal[2];
                        cd.body1 = object1->getMovable();
                        cd.body2 = object2->getMovable();
                        cd.contactMaterialObject1 = object1->getMaterialAt(cd.pos);
                        cd.contactMaterialObject2 = object2->getMaterialAt(cd.pos);
                        contactVector.push_back(cd);
                        // fprintf(stderr, "\t\tfound contact\n");
                        //  if(object1->getMovable())
                        //  {
                        //      fprintf(stderr, "\t1: %s", object1->getMovable()->getName().c_str());
                        //  }
                        //  if(object2->getMovable())
                        //  {
                        //      fprintf(stderr, "\t2: %s", object2->getMovable()->getName().c_str());
                        //  }
                        //  fprintf(stderr, "\n");
                        //  TODO: instead of creating the contact here we have to store it
                        //        in our own contact structure
                        // dJointID c=dJointCreateContact(world,contactgroup,contact+i);
                        // dJointAttach(c,b1,b2);

                        // TODO: replace num_ground_collisions with getNumContact in Object
                        // TODO: we should have the information of contact points already in the contact object
                        // contact_point.x() = contact[i].geom.pos[0];
                        // contact_point.y() = contact[i].geom.pos[1];
                        // contact_point.z() = contact[i].geom.pos[2];

                        // object1->contact_ids.push_back(object2->id);
                        // object2->contact_ids.push_back(object1->id);
                        // object1->contact_points.push_back(contact_point);
                        // object2->contact_points.push_back(contact_point);
                    }
                }
            }
            delete[] contact;
        }

        /**
         * \brief This static function is used to project a normal function
         *   pointer to a method from a class
         *
         * pre:
         *     - data is a pointer to a correct object from type CollisionSpace
         *
         * post:
         *     - the newCallback method of the data object should be called
         */
        void CollisionSpace::callbackForward(void *data, dGeomID o1, dGeomID o2)
        {
            auto* const cs = reinterpret_cast<CollisionSpace *>(data);
            cs->nearCallback(o1, o2);
        }

        int CollisionSpace::handleCollision(dGeomID theGeom)
        {
            ray_collision = 0;
            dSpaceCollide2(theGeom, (dGeomID)space, this,
                           &CollisionSpace::callbackForward);
            return ray_collision;
        }

        double CollisionSpace::getCollisionDepth(dGeomID theGeom)
        {
            dGeomID otherGeom;
            dContact contact[1];
            double depth = 0.0;
            int numc;
            dBodyID b1;
            dBodyID b2;

            for(int i=0; i<dSpaceGetNumGeoms(space); i++)
            {
                otherGeom = dSpaceGetGeom(space, i);

                if(!(dGeomGetCollideBits(theGeom) & dGeomGetCollideBits(otherGeom)))
                {
                    continue;
                }

                b1 = dGeomGetBody(theGeom);
                b2 = dGeomGetBody(otherGeom);

                if(b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact))
                {
                    continue;
                }

                numc = dCollide(theGeom, otherGeom, 1,
                                &(contact[0].geom), sizeof(dContact));
                // numc = dCollide(theGeom, otherGeom, 1 | CONTACTS_UNIMPORTANT,
                //                 &(contact[0].geom), sizeof(dContact));
                if(numc)
                {
                    if (contact[0].geom.depth > depth)
                    {
                        depth = contact[0].geom.depth;
                    }
                }
            }

            return depth;
        }

        // int CollisionSpace::checkCollisions(void) {
        //     MutexLocker locker(&iMutex);
        //     num_contacts = log_contacts = 0;
        //     create_contacts = 0;
        //     dSpaceCollide(space,this, &CollisionSpace::callbackForward);
        //     return num_contacts;
        // }

        double CollisionSpace::getVectorCollision(const Vector &pos,
                                                  const Vector &ray) const
        {
            const MutexLocker locker{&iMutex};
            dGeomID otherGeom;
            dContact contact[1];
            // double depth = ray.length();
            auto depth = ray.norm();
            int numc;

            dGeomID theGeom = dCreateRay(space, static_cast<sReal>(depth));
            dGeomRaySetClosestHit(theGeom, 1);
            dGeomRaySet(theGeom, pos.x(), pos.y(), pos.z(), ray.x(), ray.y(), ray.z());

            // TODO: first check if there is a collision with the bounding box of the space
            for(int i=0; i<dSpaceGetNumGeoms(space); i++)
            {
                otherGeom = dSpaceGetGeom(space, i);

                if(!(dGeomGetCollideBits(theGeom) & dGeomGetCollideBits(otherGeom)))
                {
                    continue;
                }
                numc = dCollide(theGeom, otherGeom, 1 | CONTACTS_UNIMPORTANT,
                                &(contact[0].geom), sizeof(dContact));
                if(numc)
                {
                    if (contact[0].geom.depth < depth)
                    {
                        depth = contact[0].geom.depth;
                    }
                }
            }

            dGeomDestroy(theGeom);
            return depth;
        }

        void CollisionSpace::getSphereCollision(const Vector &pos,
                                                const double r,
                                                std::vector<utils::Vector> &contacts,
                                                std::vector<double> &depths) const
        {
            // MutexLocker locker(&iMutex);
            // dGeomID otherGeom;
            // dContact contact[4];
            // //double depth = ray.length();
            // int numc;
            // Vector contactPos;
            // dGeomID theGeom = dCreateSphere(space, r);
            // dGeomSetPosition(theGeom, (dReal)pos.x(), (dReal)pos.y(), (dReal)pos.z());

            // for(int k=0; k<dSpaceGetNumGeoms(space); k++) {
            //     otherGeom = dSpaceGetGeom(space, k);

            //     if(!(dGeomGetCollideBits(theGeom) & dGeomGetCollideBits(otherGeom)))
            //         continue;
            //     numc = dCollide(theGeom, otherGeom, 4,
            //                     &(contact[0].geom), sizeof(dContact));
            //     for(int i=0; i<numc; ++i) {
            //         geom_data* object1 = (geom_data*)dGeomGetData(otherGeom);
            //         double filter_depth = -1.0;
            //         if(object1->filter_depth > filter_depth) {
            //             filter_depth = object1->filter_depth;
            //         }
            //         double filter_angle = 0.5;
            //         if(object1->filter_angle > 0.0) {
            //             filter_angle = object1->filter_angle;
            //         }

            //         double filter_radius = -1.0;
            //         Vector filter_sphere;
            //         if(object1->filter_radius > filter_radius) {
            //             filter_radius = object1->filter_radius;
            //             filter_sphere = object1->filter_sphere;
            //         }

            //         if(filter_depth > 0.0) {
            //             if(contact[i].geom.normal[2] < 0.5 or filter_depth < contact[i].geom.depth) {
            //                 continue;
            //             }
            //         }
            //         if(filter_radius > 0.0) {
            //             Vector v;
            //             v.x() = contact[i].geom.pos[0];
            //             v.y() = contact[i].geom.pos[1];
            //             v.z() = 0.0;//contact[i].geom.pos[2];
            //             v -= filter_sphere;
            //             if(v.norm() <= filter_radius) {
            //                 continue;
            //             }
            //         }

            //         // TODO: return contact position
            //         contactPos.x() = contact[i].geom.pos[0];
            //         contactPos.y() = contact[i].geom.pos[1];
            //         contactPos.z() = contact[i].geom.pos[2];
            //         contacts.push_back(contactPos);
            //         depths.push_back(contact[i].geom.depth);
            //     }
            // }

            // dGeomDestroy(theGeom);
        }

        ode_collision::Object *CollisionSpace::createObject(configmaps::ConfigMap &config, std::shared_ptr<interfaces::DynamicObject> movable)
        {
            std::string type = config["type"];
            if(!objects_schema.count(type))
            {
                const std::string errmsg = "CollisionSpace::createObject: could not create object " + type + ": missing schema file.";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            if(!objects_schema[type].validate(config))
            {
                const std::string errmsg = "CollisionSpace::createObject: could not create object " + type + ": invalid config.";
                LOG_ERROR("%s", errmsg.c_str());
                throw std::runtime_error(errmsg);
            }
            // TODO: what do we need here

            auto* const newObject = ObjectFactory::Instance().createObject(type, this, movable, config);
            objects[config["name"].getString()] = newObject;
            // if(movable)
            {
                dynamicObjects.push_back(newObject);
            }
            if(control->graphics)
            {
                // TODO: we leave mesh graphics here for now
                // after we get a scene loader, we dont need it anymore
                if(type == "mesh" || type == "heightfield")
                {
                    bool show = true;
                    if(control->cfg)
                    {
                        // TODO: Remove magic number
                        show = control->cfg->getOrCreateProperty("Simulator", "visual rep.", 1).iValue & 2;
                    }
                    auto map = configmaps::ConfigMap{config};
                    map["physicmode"] = config["type"];

                    // map["movable"] = true;
                    interfaces::NodeData node;
                    node.fromConfigMap(&map, "");
                    configmaps::ConfigMap material;

                    material["name"] = "collision";
                    material["diffuseColor"]["a"] = 1.0;
                    material["diffuseColor"]["r"] = 0.7;
                    material["diffuseColor"]["g"] = 0.39;
                    material["diffuseColor"]["b"] = 0.3;
                    material["specularColor"]["a"] = 1.0;
                    material["specularColor"]["r"] = 0.0;
                    material["specularColor"]["g"] = 0.0;
                    material["specularColor"]["b"] = 0.0;
                    material["ambientColor"]["a"] = 1.0;
                    material["ambientColor"]["r"] = 0.7;
                    material["ambientColor"]["g"] = 0.59;
                    material["ambientColor"]["b"] = 0.5;
                    material["shininess"] = 0.;
                    material["transparency"] = 0.3;
                    node.material.fromConfigMap(&material, "");
                    node.name += "_collision";
                    newObject->drawID = control->graphics->addDrawObject(node, show);
                    newObject->graphics = control->graphics;
                }
                else
                {
                    // This is needed, since we moved the graphics to envire_mars_graphics
                    // for the object types except mesh
                    newObject->drawID = -1;
                }
            }
            return newObject;
        }

        void CollisionSpace::updateTransforms(void)
        {
            if(control->graphics)
            {
                control->graphics->lock();
            }
            for(auto &object : dynamicObjects)
            {
                object->updateTransform();
                if(control->graphics)
                {
                    // this is required to use graphics only for mesh for now
                    if(object->drawID != -1)
                    {
                        Vector p;
                        object->getPosition(&p);
                        Quaternion q;
                        object->getRotation(&q);
                        control->graphics->setDrawObjectPos(object->drawID, p);
                        control->graphics->setDrawObjectRot(object->drawID, q);
                    }
                }
            }
            if(control->graphics)
            {
                control->graphics->unlock();
            }
        }

        void CollisionSpace::showDebugObjects(bool show)
        {
            if(control->graphics)
            {
                for(const auto &object : dynamicObjects)
                {
                    control->graphics->setDrawObjectShow(object->drawID, show);
                }
            }
        }

        void CollisionSpace::reset()
        {
            contactVector.clear();
            dynamicObjects.clear();
            objects.clear();
        }

        void CollisionSpace::swapContacts(std::vector<ContactData> &contactVector)
        {
            this->contactVector.swap(contactVector);
        }

        void CollisionSpace::getContacts(std::vector<ContactData> &contactVector)
        {
            for(const auto &it : this->contactVector)
            {
                contactVector.push_back(it);
            }
        }

        void CollisionSpace::getContacts(std::shared_ptr<CollisionInterface> other, std::vector<ContactData> &contactVector)
        {
            const MutexLocker locker{&iMutex};

            if(space_init > 0)
            {
                auto* const otherSpace = dynamic_cast<CollisionSpace*>(other.get());
                if(otherSpace)
                {
                    num_contacts = log_contacts = 0;
                    this->contactVector.clear();
                    dSpaceCollide2((dxGeom*)space, (dxGeom*)otherSpace->getSpace(), this, &CollisionSpace::callbackForward);
                    for(const auto &it : this->contactVector)
                    {
                        contactVector.push_back(it);
                    }
                }
            }
        }

        dSpaceID CollisionSpace::getSpace()
        {
            return space;
        }

        configmaps::ConfigMap CollisionSpace::getConfigMap() const
        {
            configmaps::ConfigMap result;

            configmaps::ConfigMap objectsConfigMap;
            for(const auto& object : objects)
            {
                configmaps::ConfigItem objectItem;
                objectItem = object.first;
                objectsConfigMap[object.first] = object.second->getConfigMap();
            }
            const auto objectsKey = std::string{"objects ("} + std::to_string(objects.size()) + ")";
            result[objectsKey] = objectsConfigMap;

            return result;
        }
        
        std::vector<std::string> CollisionSpace::getEditPattern(const std::string& basePath) const
        {
            return std::vector<std::string>{""};
        }

        void CollisionSpace::edit(const std::string& configPath, const std::string& value)
        {}

    } // end of namespace ode_collision
} // end of namespace mars
