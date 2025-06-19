#pragma once

#include "Vector.hpp"
#include "global.hpp"

class Object
{
public:
    Object() = default;

    virtual ~Object() = default;

    virtual bool intersect(const Vector3f&, const Vector3f&, float&, uint32_t&, Vector2f&) const = 0;

    virtual void getSurfaceProperties(const Vector3f&, const Vector3f&, const uint32_t&, const Vector2f&, Vector3f&,
                                      Vector2f&) const = 0;

    virtual Vector3f evalDiffuseColor(const Vector2f&) const
    {
        return diffuseColor;
    }

    // material properties
    MaterialType materialType = DIFFUSE_AND_GLOSSY;
    float ior = 1.3f;      // index of refraction 折射率
    float Kd = 0.8f;
    float Ks = 0.2f;
    Vector3f diffuseColor = Vector3f(0.2f);
    float specularExponent = 25.0f;
};
