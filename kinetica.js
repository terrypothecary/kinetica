//==============================================================================
// Copyright Terry Pothecary 2005 all rights reserved
//==============================================================================
var warn = 0;
var logElement;
var log = function (msg) {
    return;
    if (!logElement) {
        logElement = document.getElementById('log');
    }
    logElement.innerHTML = msg;
    //console.log.apply(console, arguments);
};
var quadraticCount = 0;
var linearCount = 0;
var countOverlap = 0;
var countStep = 0;
var countLoop = 0;
var countBounce = 0;

var Kinetica = {};
Kinetica.thingsequence = 0;

Kinetica.Space = function (domNode) {
    this.domNode = domNode;
    this.thingsThat = {};

    this.time = new Kinetica.Time();
    this.time.addSpace(this);

    this.gravity = {
        x: 0,           // Newtons / KG
        y: 9.8          // Newtons / KG
    };
    this.loss = Math.sqrt(4.0); // sqrt(Fraction of Joules / Second)
    this.scale = 0.01;          // Metres / pixel
};

Kinetica.Space.prototype = {

    //------------------------------------------------------------------------------
    addThing: function (obj) {
        this.addThingThat('draw', obj);
        this.addThingThat('updateForce', obj);
        this.addThingThat('resolveForce', obj);
        this.addThingThat('updateVelocity', obj);
        this.addThingThat('move', obj);
        return obj;
    },


    //------------------------------------------------------------------------------
    addThingThat: function (action, obj) {
        this.thingsThat[action] = this.thingsThat[action] || {things: [], actions: []};
        if (typeof(obj[action]) === 'function') {
            this.thingsThat[action].things.push(obj);
            this.thingsThat[action].actions.push(obj[action].bind(obj));
        }
        if (typeof(obj.setSpace) === 'function') {
            obj.setSpace(this);
        }

        this.addGravity(obj);
    },

    //------------------------------------------------------------------------------
    addGravity: function (obj) {
        if (typeof(obj.addForce) === 'function') {
//#tjp            obj.addForce(this.gravity);
        }
    },

    //------------------------------------------------------------------------------
    doAction: function (action, period) {
        var list = this.thingsThat[action].actions;
        for (var i = 0, n = list.length; i < n; i++) {
            list[i](period);
        }
    },

    //------------------------------------------------------------------------------
    tick: function (period) {
        countStep++;
        log('step ' + countStep + '    bounce ' + countBounce + '  loop ' + countLoop + '  quadratic ' + quadraticCount + '  linear ' + linearCount + '  overlap ' + countOverlap);

        this.doAction('draw', period);
        this.doAction('updateForce', period);
        this.doAction('resolveForce', period);
        this.doAction('updateVelocity', period);
        this.move(period);
    },

    //------------------------------------------------------------------------------
    move: function (period) {
        var localCountLoop = 0;

        var timeLeft = period;
        while (timeLeft > 0) {
            countLoop++;
            localCountLoop++;
            if (localCountLoop > 10000) {
                timeLeft = 0;
            }
            var collision = this.getNextCollision(timeLeft);

            if (collision) {
                countBounce++;
                this.doAction('move', collision.time);
                timeLeft -= collision.time;
                this.collide(collision.body1, collision.body2);
            }
            else {
                this.doAction('move', timeLeft);
                timeLeft = 0.0;
            }

        }

    },

    //------------------------------------------------------------------------------
    getNextCollision: function (period) {
        var nextTime = period;
        var collision = null;
        var bodies = this.thingsThat.move.things;

        var n = bodies.length;
        for (var i = 0; i < n; i++) {
            for (var j = i + 1; j < n; j++) {
                var body1 = bodies[i];
                var body2 = bodies[j];
                var when = this.timeToCollision(body1, body2, nextTime);
                if (when !== null) {
                    nextTime = when;
                    collision = {
                        time: nextTime,
                        body1: body1,
                        body2: body2
                    };
                }
            }
        }

        if (collision && collision.time < 0.0) {
            collision.time = 0.0;
        }

        return collision;
    },

    //------------------------------------------------------------------------------
    timeToCollision: function (body1, body2, period)
    {
        var t = [];
        var p0x = body1.px - body2.px;
        var p0y = body1.py - body2.py;
        var vx =  body1.vx - body2.vx;
        var vy =  body1.vy - body2.vy;
        var r = (body1.size + body2.size) / 2;


        // early out if the other body's x can't get to us
        t = [
            -(p0x - r) / vx,
            -(p0x + r) / vx
        ];
        if (t[0] < 0 && t[1] < 0) {
            return null;
        }
        if (t[0] > period && t[1] > period) {
            return null;
        }


        // early out if the other body's y can't get to us
        t = [
            -(p0y - r) / vy,
            -(p0y + r) / vy
        ];
        if (t[0] < 0 && t[1] < 0) {
            return null;
        }
        if (t[0] > period && t[1] > period) {
            return null;
        }


        // pt is relative position at time t
        // p0 is initial relative position
        // v is relative velocity
        // r is the sum of the two bodies' radii
        //
        // pt = p0 + t*v    => pxt = px0 + t*vx & pyt = py0 + t*vy
        // dt = |pt|        => sqrt( (pxt)^2 + (pyt)^2 )            => sqrt( (px0 + t*vx)^2 + (py0 + t*vy)^2 )
        // dt - r = 0       => dt^2 - r^2 = 0
        //
        // => (px0 + t*vx)^2 + (py0 + t*vy)^2 - r^2 = 0
        // => (px0^2 + 2*px0*t*vx + vx^2*t^2) + (py0^2 + 2*py0*t*vy + vy^2*t^2) - r^2 = 0
        // => (vx^2 + vy^2)*t^2 + (2*px0*vx + 2*py0*vy)*t + (px0^2 + py0^2) - r^2 = 0
        //
        // if a*t^2 + b*t + c = 0
        // => a = vx^2 + vy^2
        // &  b = 2*px0*vx + 2*py0*vy
        // &  c = px0^2 + py0^2 - r^2

        var a = vx * vx + vy * vy;
        var b = 2 * (p0x * vx + p0y * vy);
        var c = p0x * p0x + p0y * p0y - r * r;
        t = solveQuadratic(a, b, c);

        if (t) {
            var isCollision = false;

            var tmin = t[0];
            var tmax = t[1];

            if (tmax >= 0)
            {
                if (tmin >= 0)
                {
                    isCollision = true;
                }
                else if (tmax > -tmin)
                {
                    isCollision = true;
                    countOverlap++;
                    console.log('overlap ', body1.id, ' ', body2.id, ' ', tmin, ' ', tmax);
                }
            }

            if (isCollision && tmin < period) {
                return tmin;
            }
        }

        return null;

        function solveQuadratic(a, b, c) {
            var x = null;
            var temp = 0;

            if (a === 0) {
                if (b === 0) {
                    x = null;
                }
                else {
                    temp = -c / b;
                    x = [temp, temp];
                    linearCount++;
                }
            }
            else {
                var quadP0 = b * b - 4 * a * c;
                if (quadP0 < 0) {
                    x = null;
                }
                else {
                    var quadP1 = 2 * a;
                    var quadP2 = Math.sqrt(quadP0);
                    x = [(-b - quadP2) / quadP1, (-b + quadP2) / quadP1];
                    x.sort(function (a, b) {return a - b; });
                    quadraticCount++;
                }
            }

            return x;
        }
    },

    //------------------------------------------------------------------------------
    collide: function (body1, body2) {
        var m1 = body1.mass;
        var p1x = body1.px;
        var p1y = body1.py;
        var v1x = body1.vx;
        var v1y = body1.vy;

        var m2 = body2.mass;
        var p2x = body2.px;
        var p2y = body2.py;
        var v2x = body2.vx;
        var v2y = body2.vy;

        // Find direction normal (n) to both bodies from the centre of body1 to centre of body2
        var nx = p2x - p1x;
        var ny = p2y - p1y;
        var nd = Math.sqrt(nx * nx + ny * ny);
        nx = nx / nd;
        ny = ny / nd;

        // Find the direction perpendicular (q) to the normal
        var qx = -ny;
        var qy = nx;

        // Determine the components of v1 in n and q
        var v1n = (v1x * nx) + (v1y * ny);
        var v1q = (v1x * qx) + (v1y * qy);

        // Determine the components of v2 in n and q
        var v2n = (v2x * nx) + (v2y * ny);
        var v2q = (v2x * qx) + (v2y * qy);

        // Only the velocities in the direction of n are affected by the collision

        // determine the reference velocity that gives total zero momentum in the direction of n
        var i1 = v1n * m1;
        var i2 = v2n * m2;
        var vref = (i1 + i2) / (m1 + m2);

        // each body's velocity is negated with respect to the reference velocity
        v1n = 2 * vref - v1n;
        v2n = 2 * vref - v2n;

        // Resolve velocities back in the x and y axis
        body1.vx = v1n * nx + v1q * qx;
        body1.vy = v1n * ny + v1q * qy;
        body2.vx = v2n * nx + v2q * qx;
        body2.vy = v2n * ny + v2q * qy;
    }

};


//==============================================================================
Kinetica.Time = function (period) {
    this.period = 0.01;
    this.timerID = 0;
    this.spaces = [];
};

Kinetica.Time.prototype = {

    //------------------------------------------------------------------------------
    setPeriod: function (period) {
        this.period = period;
    },

    //------------------------------------------------------------------------------
    addSpace: function (space) {
        this.spaces.push(space);
    },

    //------------------------------------------------------------------------------
    start: function () {
        if (this.timerID !== 0) {
            this.Stop();
        }
        this.timerID = setInterval(this.tick.bind(this), this.period * 1000);
    },

    //------------------------------------------------------------------------------
    stop: function () {
        clearInterval(this.timerID);
        this.timerID = 0;
    },

    //------------------------------------------------------------------------------
    tick: function () {
        for (var i = 0, n = this.spaces.length; i < n; i++) {
            this.spaces[i].tick(this.period);
        }
    }
};


//==============================================================================
Kinetica.Point = function (size, mass, image) {
    this.id = Kinetica.thingsequence++;

    this.px = 0.0;  // Position [x]
    this.py = 0.0;  // Position [y]
};

Kinetica.Point.prototype = {
};


//==============================================================================
Kinetica.Body = function (size, mass, image) {
    this.id = Kinetica.thingsequence++;

    this.size = size;           // diameter in Metres
    this.mass = mass;           // KG
    this.image = image;

    this.forces = [];
    this.space = null;
    this.element = null;

    this.px = 0.0;  // Position [x]
    this.py = 0.0;  // Position [y]
    this.vx = 0.0;  // Velocity [x]
    this.vy = 0.0;  // Velocity [y]
};

Kinetica.Body.prototype = {
    //------------------------------------------------------------------------------
    setSpace: function (space) {
        this.space = space;
    },

    //------------------------------------------------------------------------------
    draw: function () {

        if (!this.element && this.image) {
            var ImageSize = this.size / this.space.scale;
            var elementId = 'kinetica-' + this.id;
            var node = document.createElement('div');
            node.innerHTML = '<div id="' + elementId + '" style="position:absolute; top:0; left:0;"><img src="' + this.image + '" width="' + ImageSize + 'px" height="' + ImageSize + 'px" /></div>';
            this.element = node.childNodes[0];
            this.space.domNode.appendChild(this.element);
        }

        if (this.element) {
            var offset = this.size * 0.5;
            var pixelx = Math.round((this.px - offset) / this.space.scale);
            var pixely = Math.round((this.py - offset) / this.space.scale);
            if (pixelx !== this.pixelx) {
                this.pixelx = pixelx;
                this.element.style.left = pixelx + "px";
            }
            if (pixely !== this.pixely) {
                this.pixely = pixely;
                this.element.style.top = pixely + "px";
            }
        }
    },

    //------------------------------------------------------------------------------
    addForce: function (force) {
        this.forces.push(force);
    },

    //------------------------------------------------------------------------------
    resolveForce: function () {
        var forces = this.forces;
        this.fx = this.mass * this.space.gravity.x;
        this.fy = this.mass * this.space.gravity.y;
        for (var i = 0, n = forces.length; i < n; i++) {
            var force = forces[i];
            this.fx += force.x;
            this.fy += force.y;
        }
    },

    //------------------------------------------------------------------------------
    updateVelocity: function (period) {
        this.vx += this.fx / this.mass * period;
        this.vy += this.fy / this.mass * period;

        this.vx *= (1.0 - this.space.loss * period);
        this.vy *= (1.0 - this.space.loss * period);
    },

    //------------------------------------------------------------------------------
    move: function (period) {
        this.px += this.vx * period;
        this.py += this.vy * period;
    }

};


//==============================================================================
Kinetica.Link = function (point1, point2, length, extensionModulus, compressionModulus)
{
    if (length === undefined) {
        length = 1.0;
    }
    if (extensionModulus === undefined) {
        extensionModulus = 60;
    }
    if (compressionModulus === undefined) {
        compressionModulus = extensionModulus;
    }
    this.point1 = point1;                           // Metres
    this.point2 = point2;                           // Metres
    this.naturalLength = length;                     // Metres
    this.extensionModulus = extensionModulus;       // Newtons / Metre
    this.compressionModulus = compressionModulus;   // Newtons / Metre

    this.force1 = {
        x: 0,
        y: 0
    };
    this.force2 = {
        x: 0,
        y: 0
    };

    if (typeof(this.point1.addForce) === 'function') {
        this.point1.addForce(this.force1);
    }
    if (typeof(this.point2.addForce) === 'function') {
        this.point2.addForce(this.force2);
    }
};

Kinetica.Link.prototype = {
//------------------------------------------------------------------------------
    updateForce: function () {
        var dx = (this.point2.px - this.point1.px);
        var dy = (this.point2.py - this.point1.py);
        var s = Math.sqrt(dx * dx + dy * dy);
        var nx = dx / s;
        var ny = dy / s;

        var fx = 0;
        var fy = 0;
        if (s >= this.naturalLength)
        {
            fx += nx * (s - this.naturalLength) * this.extensionModulus;
            fy += ny * (s - this.naturalLength) * this.extensionModulus;
        }
        else {
            fx += nx * (s - this.naturalLength) * this.compressionModulus;
            fy += ny * (s - this.naturalLength) * this.compressionModulus;
        }

        this.force1.x = fx;
        this.force1.y = fy;

        this.force2.x = -fx;
        this.force2.y = -fy;
    }
};


//==============================================================================
Kinetica.Mouse = function () {
    this.px = 0;
    this.py = 0;
    document.body.addEventListener('mousemove', this.onmousemove.bind(this), false);
    document.body.addEventListener('click', this.onclick.bind(this), false);
};

Kinetica.Mouse.prototype = {
    //------------------------------------------------------------------------------
    setSpace: function (space) {
        this.space = space;
    },

    onmousemove: function (evt) {
        this.px = evt.clientX * this.space.scale;
        this.py = evt.clientY * this.space.scale;
    },

    onclick: function (evt) {
        this.px = evt.clientX * this.space.scale;
        this.py = evt.clientY * this.space.scale;
    }

};

