#include "Heightfield.hpp"
#include <mars_interfaces/terrainStruct.h>

namespace mars
{
    namespace ode_collision
    {

        using namespace utils;
        using namespace interfaces;
        using namespace configmaps;

        Heightfield::Heightfield(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap& config) : 
            Object(space, movable, config),
            height_data{nullptr},
            terrain{nullptr}
        {}

        Heightfield::~Heightfield(void)
        {
            if(height_data)
            {
                free(height_data);
            }
            if(terrain)
            {
                if(terrain->pixelData)
                {
                    free(terrain->pixelData);
                    terrain->pixelData = nullptr;
                }
                delete terrain;
            }
        }

        Object* Heightfield::instantiate(CollisionInterface* space, std::shared_ptr<DynamicObject> movable, ConfigMap& config)
        {
            return static_cast<Object*>(new Heightfield{space, movable, config});
        }

        dReal heightfield_callback(void* pUserData, int x, int z)
        {
            return static_cast<Heightfield*>(pUserData)->heightCallback(x, z);
        }

        dReal Heightfield::heightCallback(int x, int y)
        {
            return static_cast<dReal>(height_data[(y*terrain->width)+x]*terrain->scale);
        }

        void Heightfield::setTerrainStruct(interfaces::terrainStruct* t)
        {
            terrain = t;
        }

        bool Heightfield::createGeom()
        {
            dMatrix3 R;
            unsigned long size;
            int x, y;
            size = terrain->width*terrain->height;
            if(!height_data)
            {
                height_data = (dReal*)calloc(size, sizeof(dReal));
            }
            for(x=0; x<terrain->height; x++)
            {
                for(y=0; y<terrain->width; y++)
                {
                    height_data[(terrain->height-(x+1))*terrain->width+y] = static_cast<dReal>(terrain->pixelData[x*terrain->width+y]);
                }
            }
            // build the ode representation
            const auto heightid = dGeomHeightfieldDataCreate();

            // Create an finite heightfield.
            dGeomHeightfieldDataBuildCallback(heightid, this, heightfield_callback,
                                              terrain->targetWidth,
                                              terrain->targetHeight,
                                              terrain->width, terrain->height,
                                              REAL(1.0), REAL( 0.0 ),
                                              REAL(1.0), 0);
            // Give some very bounds which, while conservative,
            // makes AABB computation more accurate than +/-INF.
            dGeomHeightfieldDataSetBounds(heightid, REAL(-terrain->scale*2.0),
                                          REAL(terrain->scale*2.0));
            //dGeomHeightfieldDataSetBounds(heightid, -terrain->scale, terrain->scale);
            nGeom = dCreateHeightfield(space->getSpace(), heightid, 1);
            dRSetIdentity(R);
            dRFromAxisAndAngle(R, 1, 0, 0, M_PI/2);
            dGeomSetRotation(nGeom, R);
            if(config.hasKey("coll_bitmask"))
            {
                c_params.coll_bitmask = config["coll_bitmask"];
            }
            dGeomSetData(nGeom, this);
            objectCreated = true;
            name << config["name"];
            return true;
        }

        void Heightfield::updateTransform(void)
        {
            // local transform is relative to parent frame
            //fprintf(stderr, "update collision of ");
            // currently heightmaps are always static
            dGeomSetPosition(nGeom, static_cast<dReal>(pos.x()),
                             static_cast<dReal>(pos.y()), static_cast<dReal>(pos.z()));
            // todo: handle orientation
        }


        void Heightfield::getRotation(utils::Quaternion* q) const
        {
            *q = Quaternion::Identity();
            // q->w() = 1.0;
            // q->x() = q->y() = q->z() = 0.0;
        }

    } // end of namespace ode_collision
} // end of namespace mars
