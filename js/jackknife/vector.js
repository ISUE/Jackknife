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

/**
 * This class is a wrapper around the std::vector class that
 * provides some mathematics support for points and vectors.
 */
var Vector = function(x, size) {
    this.length = size || 0;

    /**
     * Initialize an m-component vector, setting each
     * component to a constant value.
     */
    if ((size > 0) && (Number.isInteger(x) ||
            x == Number.POSITIVE_INFINITY ||
            x == Number.NEGATIVE_INFINITY)) {
        this.data = new Array(size)
        for (var ii = 0; ii < size; ii++) {
            this.data[ii] = x;
        }
    }
    /**
     * Create an uninitialized m-component vector.
     */
    else if (Number.isInteger(x)) {
        this.data = new Array(x);
        this.length = x;
    }
    /**
     * Initialize a new vector as a copy of another vector.
     */
    else if (x instanceof Vector) {
        this.data = x.data;
        this.length = x.data.length;
    }
    /**
     * Initialize a new vector from an existing data array.
     */
    else if (x instanceof Array) {
        this.data = new Array(x.length);
        for (var ii = 0; ii < x.length; ii++) {
            this.data[ii] = x[ii];
        }
        this.length = x.length;
    }
}

/**
 * Interpolate between two vectors, where 0 <= t <= 1,
 * t = 0 = a, and t = 1 = b.
 */
function InterpolateVectors(a, b, t) {
    var m = a.length;
    var n = b.length;

    console.assert(m == n, 'Different sized arrays to interpolate');

    var data = new Array(m);
    for (var ii = 0; ii < m; ii++) {
        data[ii] = (1.0 - t) * a.data[ii];
        data[ii] += t * b.data[ii];
    }

    return new Vector(data);
}

Vector.prototype.size = function() {
    return this.length;
}

Vector.prototype.elementAt = function(idx) {
    return this.data[idx];
}

Vector.prototype.setAllElementsTo = function(rhs) {
    for (var ii = 0; ii < this.length; ii++) {
        this.data[ii] = rhs;
    }
}

Vector.prototype.negative = function() {
    var m = this.length;
    var vec = new Vector(m);

    for (var ii = 0; ii < m; ii++) {
        vec.data[ii] = -this.data[ii];
    }

    return vec;
}

Vector.prototype.add = function(rhs) {
    var m = this.length;
    var vec = new Vector(m);

    for (var ii = 0; ii < m; ii++) {
        vec.data[ii] = this.data[ii] + rhs.data[ii];
    }

    return vec;
}

Vector.prototype.subtract = function(rhs) {
    var m = this.length;
    var vec = new Vector(m);

    for (var ii = 0; ii < m; ii++) {
        vec.data[ii] = this.data[ii] - rhs.data[ii];
    }

    return vec;
}

Vector.prototype.divide = function(rhs) {
    if (rhs instanceof Vector) {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] / rhs.data[ii];
        }

        return vec;
    } else {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] / rhs;
        }

        return vec;
    }
}

Vector.prototype.multiply = function(rhs) {
    if (rhs instanceof Vector) {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] * rhs.data[ii];
        }

        return vec;
    } else {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] * rhs;
        }

        return vec;
    }
}

Vector.prototype.equals = function(rhs) {
    var m = this.length;
    var ret = 1;

    for (var ii = 0; ii < m; ii++) {
        ret &= (this.data[ii] == rhs.data[ii]);
    }

    return ret;
}

Vector.prototype.l2norm2 = function(other) {
    var ret = 0;

    for (var ii = 0; ii < this.length; ii++) {
        var delta = this.data[ii] - other.data[ii];
        ret += delta * delta;
    }

    return ret;
}

Vector.prototype.l2norm = function(other) {
    return Math.sqrt(this.l2norm2(other));
}

Vector.prototype.magnitude = function() {
    var ret = 0;
    for (var ii = 0; ii < this.length; ii++) {
        ret += this.data[ii] * this.data[ii];
    }

    return Math.sqrt(ret);
}

Vector.prototype.normalize = function() {
    var len = this.magnitude();

    for (var ii = 0; ii < this.length; ii++) {
        this.data[ii] = this.data[ii] / len;
    }
    return this;
}

Vector.prototype.dot = function(rhs) {
    var m = this.length;
    var ret = 0;

    for (var ii = 0; ii < m; ii++) {
        ret += this.data[ii] * rhs.data[ii];
    }

    return ret;
}

Vector.prototype.sum = function() {
    var ret = 0;

    for (var ii = 0; ii < this.length; ii++) {
        ret += this.data[ii];
    }

    return ret;
}

Vector.prototype.cumulative_sum = function() {
    var ret = 0;

    for (var ii = 0; ii < this.data.length; ii++) {
        ret += this.data[ii];
        this.data[ii] = ret;
    }

}
