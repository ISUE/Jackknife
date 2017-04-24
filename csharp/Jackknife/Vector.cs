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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Jackknife
{
    /**
     * This class is a wrapper around the std::vector class that
     * provides some mathematics support for points and vectors.
     */
    public class Vector : IEquatable<Vector>, ICloneable
    {
        public List<double> Data { get; set; }

        /**
         * Return component count.
         */
        public int Size
        {
            get
            {
                return Data.Count;
            }
        }

        /**
        * Create an uninitialized m-component vector.
        */
        public Vector(int m) : this(0, m) { }

        /**
         * Initialize an m-component vector, setting each
         * component to a constant value.
         */
        public Vector(double constant, int m)
        {
            Data = new List<double>();

            for (int i = 0; i < m; i++)
                Data.Add(constant);
        }

        /**
         * Initialize a new component as a copy of another vector.
         */
        public Vector(List<double> other)
        {
            int m = other.Count;

            Data = new List<double>();

            for (int i = 0; i < m; i++)
                Data.Add(other[i]);
        }

        /**
         * Interpolate between two vectors, where 0 <= t <= 1,
         * t = 0 = a, and t = 1 = b.
         */
        public Vector(Vector a, Vector b, double t)
        {
            int m = a.Data.Count;

            if (a.Data.Count != b.Data.Count)
                throw new ArgumentException("The size of the two vectors must be equal");

            Data = new List<double>();

            for (int i = 0; i < m; i++)
            {
                double d = (1.0 - t) * a.Data[i];
                d += t * b.Data[i];
                Data.Add(d);
            }
        }

        /**
         * Make the vector easily indexable.
         */
        public double this[int index]
        {
            get
            {
                return Data[index];
            }

            set
            {
                Data[index] = value;
            }
        }

        /**
         * Assign each component to a constant.
         */
        public void Set(double rhs)
        {
            for (int i = 0; i < Size; i++)
                Data[i] = rhs;
        }

        public static Vector operator -(Vector vec)
        {
            Vector result = new Vector(vec.Size);

            for (int i = 0; i < result.Size; i++)
                result[i] = -vec[i];

            return result;
        }

        public static Vector operator *(Vector vec, double lhs)
        {
            Vector result = new Vector(vec.Size);

            for (int i = 0; i < result.Size; i++)
                result[i] = vec[i] * lhs;

            return result;
        }

        public static Vector operator /(Vector vec, double lhs)
        {
            Vector result = new Vector(vec.Size);

            for (int i = 0; i < result.Size; i++)
                result[i] = vec[i] / lhs;

            return result;
        }

        public static Vector operator /(Vector vec, Vector vec2)
        {
            Vector result = new Vector(vec.Size);

            for (int i = 0; i < result.Size; i++)
                result[i] = vec[i] / vec2[i];

            return result;
        }

        public static Vector operator +(Vector vec, Vector lhs)
        {
            Vector result = new Vector(vec.Size);

            for (int i = 0; i < result.Size; i++)
                result[i] = vec[i] + lhs[i];

            return result;
        }

        public static Vector operator -(Vector vec, Vector lhs)
        {
            Vector result = new Vector(vec.Size);

            for (int i = 0; i < result.Size; i++)
                result[i] = vec[i] - lhs[i];

            return result;
        }

        public static bool operator ==(Vector left, Vector right)
        {
            return left.Equals(right);
        }

        public static bool operator !=(Vector left, Vector right)
        {
            return !left.Equals(right);
        }

        public override int GetHashCode()
        {
            int result = this[0].GetHashCode();

            for (int i = 1; i < Size; i++)
                result ^= this[i].GetHashCode();

            return result;
        }

        /**
         * Euclidean distance squared.
         */
        public double L2Norm2(Vector other)
        {
            double ret = 0;

            for (int i = 0; i < Size; i++)
            {
                double delta = Data[i] - other[i];
                ret += delta * delta;
            }

            return ret;
        }

        /**
         * Euclidean distance.
         */
        public double L2Norm(Vector other)
        {
            return Math.Sqrt(L2Norm2(other));
        }

        public double Length()
        {
            double ret = 0;

            for (int i = 0; i < Size; i++)
                ret += Data[i] * Data[i];

            return Math.Sqrt(ret);
        }

        public Vector Normalize()
        {
            double length = Length();

            for (int i = 0; i < Size; i++)
            {
                Data[i] /= length;
            }

            return this;
        }

        public double Dot(Vector rhs)
        {
            double ret = 0;

            for (int i = 0; i < Size; i++)
                ret += Data[i] * rhs[i];

            return ret;
        }

        public double Sum()
        {
            double ret = 0;

            for (int i = 0; i < Size; i++)
                ret += Data[i];

            return ret;
        }

        public void CumulativeSum()
        {
            double sum = 0;

            for (int i = 0; i < Size; i++)
            {
                sum += Data[i];
                Data[i] = sum;
            }
        }

        public bool Equals(Vector other)
        {
            for (int i = 0; i < Size; i++)
                if (this[i] != other[i])
                    return false;

            return true;
        }

        public override bool Equals(object other)
        {
            if (!(other is Vector))
                return false;

            Vector temp = other as Vector;
            return Equals(temp);
        }

        public object Clone()
        {
            return new Vector(this.Data);
        }
    }
}
