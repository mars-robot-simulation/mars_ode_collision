/**
 * \file CollisionSpace.hpp
 * \author Malte Langosz and Team
 * \brief "CollisionSpace" includes the methods to handle the physically world.
 *
 */

#pragma once
#include <mars_utils/Mutex.h>
#include <mars_utils/Vector.h>
#include <mars_interfaces/sim_common.h>
#include <mars_interfaces/ContactData.hpp>
#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_interfaces/sim/CollisionInterface.hpp>
#include <data_broker/DataBrokerInterface.h>

#include <vector>

#include <ode/ode.h>

//#include "ContactsPhysics.hpp"

namespace mars
{
    namespace ode_collision
    {

        class Object;

        /**
         * Declaration of the physical class, that implements the
         * physics interface.
         */
        class CollisionSpace : public interfaces::CollisionInterface
        {
        public:
            CollisionSpace(interfaces::ControlCenter *control);
            virtual ~CollisionSpace(void) override;
            virtual void initSpace(void) override;
            virtual void freeSpace(void) override;
            virtual void generateContacts(void) override;
            virtual void swapContacts(std::vector<interfaces::ContactData> &contactVector) override;
            virtual void getContacts(std::vector<interfaces::ContactData> &contactVector) override;
            virtual void getContacts(std::shared_ptr<CollisionInterface> other, std::vector<interfaces::ContactData> &contactVector) override;

            //virtual void swapContacts(std::vector<interfaces::ContactData> &contactVector) override;
            virtual bool existsSpace(void) const override;
            virtual interfaces::sReal getVectorCollision(const utils::Vector &pos, const utils::Vector &ray) const override;
            virtual void getSphereCollision(const utils::Vector &pos,
                                            const double r,
                                            std::vector<utils::Vector> &contacts,
                                            std::vector<double> &depths) const override;
            virtual ode_collision::Object* createObject(configmaps::ConfigMap &config, std::shared_ptr<interfaces::DynamicObject> movable=nullptr) override;
            virtual void updateTransforms(void) override;
            virtual void showDebugObjects(bool show) override;
            virtual void reset() override;

            // --- mars::interfaces::ConfigMapInterface ---
            virtual configmaps::ConfigMap getConfigMap() const override;
            virtual std::vector<std::string> getEditPattern(const std::string& basePath) const override;
            virtual void edit(const std::string& configPath, const std::string& value) override;

            void registerSchemaValidators();

            int handleCollision(dGeomID theGeom);
            interfaces::sReal getCollisionDepth(dGeomID theGeom);
            dSpaceID getSpace();

            mutable utils::Mutex iMutex;
            dReal max_angular_speed;
            dReal max_correcting_vel;

        private:
            utils::Mutex drawLock;
            dSpaceID space;
            bool space_init;
            std::vector<interfaces::ContactData> contactVector;
            interfaces::ControlCenter *control;
            std::map<std::string, configmaps::ConfigSchema> objects_schema;
            // NOTE: The Object* are deleted by removing the shared_ptr<Object> from the envireGraph in core::CollisionManager::clear.
            std::map<std::string, Object*> objects;
            std::vector<Object*> dynamicObjects;

            bool create_contacts, log_contacts;
            int num_contacts;
            int ray_collision;
            // this functions are for the collision implementation
            void nearCallback (dGeomID o1, dGeomID o2);
            static void callbackForward(void *data, dGeomID o1, dGeomID o2);

            // Step the World auxiliar methods
            void preStepChecks(void);
            void clearPreviousStep(void);
        };

    } // end of namespace ode_collision
} // end of namespace mars

