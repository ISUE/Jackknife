/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 * 
 *     Eugene M. Taranta II <etaranta@gmail.com>
 *     Amirreza Samiei <samiei@knights.ucf.edu>
 *     Mehran Maghoumi <mehran@cs.ucf.edu>
 *     Pooya Khaloo <pooya@cs.ucf.edu>
 *     Corey R. Pittman <cpittman@knights.ucf.edu>
 *     Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 * 
 * Subject to the terms and conditions of the Florida Public Educational
 * Institution non-exclusive software license, this software is distributed 
 * under a non-exclusive, royalty-free, non-sublicensable, non-commercial, 
 * non-exclusive, academic research license, and is distributed without warranty
 * of any kind express or implied. 
 *
 * The Florida Public Educational Institution non-exclusive software license
 * is located at <https://github.com/ISUE/Jackknife/blob/master/LICENSE>.
 */

#pragma once

#include <cmath>
#include <vector>
#include <assert.h>

namespace Jackknife {

/**
 * This class is a wrapper around the std::vector class that
 * provides some mathematics support for points and vectors.
 */
class Vector {

public:

    std::vector<double> data;

    /**
     *
     */
    Vector()
    {
    }

    /**
     * Create an uninitialized m-component vector.
     */
    Vector(int m)
    {
        data.resize(m);
    }

    /**
     * Initialize an m-component vector, setting each
     * component to a constant value.
     */
    Vector(
        double constant,
        int m)
    {
        data.resize(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            data[ii] = constant;
        }
    }

    /**
     * Initialize a new component as a copy of another vector.
     */
    Vector(const std::vector<double> &other)
    {
        int m = other.size();
        data.resize(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            data[ii] = other[ii];
        }
    }

    /**
     * Interpolate between two vectors, where 0 <= t <= 1,
     * t = 0 = a, and t = 1 = b.
     */
    Vector(
        const Vector &a,
        const Vector &b,
        double t)
    {
        int m = a.data.size();
        assert(m == b.data.size());

        data.resize(m);

        for(int ii = 0;
            ii < m;
            ii++)
        {
            data[ii] = (1.0 - t) * a.data[ii];
            data[ii] += t * b.data[ii];
        }
    }

    /**
     * Return component count.
     */
    int size(void) const
    {
        return data.size();
    }

    /**
     * Make the vector easily indexable.
     */
    double& operator[](const int idx)
    {
        return data[idx];
    }

    /**
     * Assign each component to a constant.
     */
    Vector& operator=(const double rhs)
    {
        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            data[ii] = rhs;
        }

        return *this;
    }

    /**
     *
     */
    Vector operator-() const
    {
        int m = data.size();
        Vector pt(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            pt.data[ii] = -data[ii];
        }

        return pt;
    }

    /**
     *
     */
    Vector& operator/=(const double lhs)
    {
        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            data[ii] /= lhs;
        }

        return *this;
    }

    /**
     *
     */
    Vector& operator+=(const Vector& lhs)
    {
        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            data[ii] += lhs.data[ii];
        }

        return *this;
    }

    /**
     *
     */
    Vector& operator-=(const Vector& lhs)
    {
        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            data[ii] -= lhs.data[ii];
        }

        return *this;
    }

    /**
     *
     */
    Vector operator+(const Vector &rhs) const
    {
        int m = data.size();
        Vector pt(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            pt.data[ii] = data[ii] + rhs.data[ii];
        }

        return pt;
    }

    /**
     *
     */
    Vector operator-(const Vector &rhs) const
    {
        int m = data.size();
        Vector pt(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            pt.data[ii] = data[ii] - rhs.data[ii];
        }

        return pt;
    }

    /**
     *
     */
    Vector operator/(const double &rhs) const
    {
        int m = data.size();
        Vector pt(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            pt.data[ii] = data[ii] / rhs;
        }

        return pt;
    }

    /**
     *
     */
    Vector operator*(const double &rhs) const
    {
        int m = data.size();
        Vector pt(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            pt.data[ii] = data[ii] * rhs;
        }

        return pt;
    }

    /**
     *
     */
    Vector operator/(const Vector &rhs) const
    {
        int m = data.size();
        Vector pt(m);

        for (int ii = 0;
             ii < m;
             ii++)
        {
            pt.data[ii] = data[ii] / rhs.data[ii];
        }

        return pt;
    }

    /**
     *
     */
    bool operator ==(const Vector &p) const
    {
        int m = data.size();
        int ret = 1;

        for (int ii = 0;
             ii < m;
             ii++)
        {
            ret &= data[ii] == p.data[ii];
        }

        return ret;
    }

    /**
     * Euclidean distance squared.
     */
    double l2norm2(const Vector &other) const
    {
        double ret = 0.0;

        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            double delta = data[ii] - other.data[ii];
            ret += delta * delta;
        }

        return ret;
    }

    /**
     * Euclidean distance.
     */
    double l2norm(const Vector &other) const
    {
        return sqrt(l2norm2(other));
    }

    /**
     *
     */
    double length(void) const
    {
        double ret = 0.0;

        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            ret += data[ii] * data[ii];
        }

        return sqrt(ret);
    }

    /**
     *
     */
    Vector& normalize(void)
    {
        double len = length();

        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            data[ii] /= len;
        }

        return *this;
    }

    /**
     *
     */
    double dot(const Vector &rhs) const
    {
        double ret = 0.0;

        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            ret += data[ii] * rhs.data[ii];
        }

        return ret;
    }

    /**
     *
     */
    double sum(void) const
    {
        double ret = 0.0;

        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            ret += data[ii];
        }

        return ret;
    }

    /**
     * Cumulative sum of vector components.
     */
    void cumulative_sum(void)
    {
        double sum = 0.0;
        for (int ii = 0;
             ii < data.size();
             ii++)
        {
            sum += data[ii];
            data[ii] = sum;
        }
    }
};

} // Jackknife namespace
