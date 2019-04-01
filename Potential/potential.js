// Static potential field class
class StaticPotentialField {
    constructor() {
        this.width = width;
        this.height = height;

        this.field = [];

        this.highest_value = 0;

        this.default = 5000;

        this.reset();
    }

    // reset field to default
    reset() { 
        for(var j = 0; j < this.height; j++)
            for(var i = 0; i < this.width; i++)
                this.field[i+j*this.width] = this.default;
    }

    // set an area to zero
    clear(x, y, w, h) { 
        x = Math.round(x);
        y = Math.round(y);
        w = Math.round(w);
        h = Math.round(h);

        for(var j = y; j < y+h; j++)
            for(var i = x; i < x+w; i++)
                this.field[i+j*this.width] = 0;
    }

    // get potential value at coordinate
    get(x, y) {
        x = Math.round(x);
        y = Math.round(y);

        if(x >=0 && y >= 0 && x < this.width && y < this.height)
            return this.field[x+y*this.width];
        else
            return this.default;
    }

    // average neighbor
    getAverageNeighbor(x, y) {
        let sum = 0; let amount = 0;
        let ax = [-1, 0, 1, 1, 1, 0, -1, -1];
        let ay = [-1, -1, -1, 0, 1, 1, 1, 0];

        for(let j = 0; j < 9; j++) {
            let neighbor = this.get(x+ax[j], y + ay[j]);
            if(neighbor != this.default) {
                sum += neighbor;
                amount++;
            }
        }
        return sum/amount;
    }

    // calculate highest value in the potential field
    getHighestValue() {
        let value = 0;
        for(let y = 0; y < this.height; y++) {
            for(let x = 0; x < this.width; x++) {
                value = Math.max(value, this.field[x+y*this.width]);
            }
        }
        return this.highest_value = value;
    }

    // set value
    set(x, y, value) {
        if(isNaN(x) || isNaN(y)) return;
        if(x < 0 || y < 0 || x > this.width || y > this.height) return;
        if(value < 0 || isNaN(value) || value <= 0) return;

        x = Math.round(x);
        y = Math.round(y);

        this.field[x+y*this.width] = value;
    }

    // set value in the potential field, set to min between current and new
    set_min(x, y, value) {
        this.set(x, y, Math.max(Math.min(value, this.get(x, y)), 0));
    }
    
    // set value in the potential field, set to max between current and new
    set_max(x, y, value) {
        this.set(x, y, Math.min(Math.max(value, this.get(x, y)), this.default));
    }

    // set a repulsive potential field for a circle
    setRepulsionCircle(x, y, r, influence, eta) {
        if(eta == null) eta = 35000/20*influence;

        for(let j = 0; j < this.height; j++) {
            for(let i = 0; i < this.width; i++) {
                let d = Math.sqrt(Math.pow(x-i,2) + Math.pow(y-j,2));
                if(d < (r+influence)) {
                    if(d < r) {
                        this.set_max(i, j, this.default-1);
                    } else {
                        let potential = Math.min(0.5*eta*(1/(d-r)+1/(influence))^2, this.default-1);
                        this.set_max(i, j, potential);
                    }
                }
            }
        }
    }

    // set repulsion potential field for a rectangle
    setRepulsionRectangle(x, y, w, h, influence, eta) {
        if(eta == null) eta = 35000/20*influence; // tuning parameter

        let rho  = 0;
        for (let rx = 0; rx < this.width; rx++) {
            for (let ry = 0; ry < this.height; ry++) {
                if (ry<y && rx<x)
                    rho = this.distance(rx,x,ry,y);
                else if (rx >= x && rx <= x+w && ry<y)
                    rho = y-ry;
                else if (rx>x+w && ry<y)
                    rho = this.distance(rx,x+w,ry,y);
                else if (rx > x+w && ry >= y && ry <= y+h)
                    rho = rx-(x+w);
                else if (rx > x+w && ry> y+h)
                    rho = this.distance(rx,x+w,ry,y+h);
                else if (rx >= x && rx <= x+w && ry > y+h)
                    rho  = ry-(y+h);
                else if (rx < x && ry > y+h)
                    rho = this.distance(rx,x,ry,y+h);
                else if (rx < x && ry >= y && ry <= y+h)
                    rho = x-rx;

                if(rx > x && rx < (x+w) && ry > y && ry < (y+h)) 
                    this.set_max(rx, ry, this.default-1);
                if (rho <= influence && rho > 0) {
                    let pot = Math.min(0.5*eta*(1/(rho)+1/(influence))^2,this.default-1);
                    this.set_max(rx, ry, pot);
                }
            }
        }
    }

    // calculate distance between two positions
    distance(x1,x2,y1,y2) {
        let dist = Math.sqrt(Math.pow(x1-x2,2)+Math.pow(y1-y2,2));
        return dist;
    }

    // go through the field and assume lowest neighbor value
    smudge(map) {
        for(let y = 0; y < this.height; y++) {
            for(let x = 0; x < this.width; x++) {
                if(map.checkPointCollision(x, y)) continue;

                let average_neighbor = this.getAverageNeighbor(x, y);
                if(average_neighbor != 0) this.set(x, y, average_neighbor);
            }
        }
    }

    // draw potential field
    draw(ctx) {
        this.getHighestValue();

        for(var j = 0; j < this.height; j++) {
            for(var i = 0; i < this.width; i++) {
                let value = this.get(i, j);

                // calculate color for gradient
                let scaled_cost = (value/this.highest_value)*2;
                let b = 0; let g = 0; let r = 0;

                if(scaled_cost < 1) {
                    b = (1-scaled_cost)*255;
                    g = scaled_cost*255;
                } else {
                    g = (1 - (scaled_cost - 1))*255;
                    r = (scaled_cost - 1)*255;
                }

                ctx.fillStyle = 'rgba(' + r+ ', ' + g + ',' + b + ',0.6)';
                ctx.fillRect(i, j, 1, 1);
            }
        }
    }
}

// Dynamic potential field class
class DynamicPotentialField extends StaticPotentialField {
    constructor(static_field, self, robots) {
        super();

        this.static_field = static_field;
        this.self = self;
        this.robots = robots;

        this.radius_factor = 1;
        this.influence = 25;

        this.clear(0, 0, this.width, this.height);
    }

    // update the dynamic be empty other that the repulsion potential fields for the other robots
    update() {
        let s = this.self.radius*this.radius_factor+this.influence+5;
        this.clear(this.self.x - s, this.self.y - s, 2*s, 2*s);
        
        for(var i = 0; i < this.robots.length; i++) {
            let robot = this.robots[i];
            if(robot != this.self) {
                this.setRepulsionCircle(robot.x, robot.y, robot.radius*this.radius_factor, this.influence, 50000);
            }
        }
    }

    // draw dynamic potential field
    drawFast(ctx) {
        for(var i = 0; i < this.robots.length; i++) {
            let robot = this.robots[i];
            if(robot != this.self) {
                ctx.lineWidth = 1;
                ctx.strokeStyle = '#FFFFFF';
                ctx.beginPath();
                ctx.arc(robot.x, robot.y, robot.radius*this.radius_factor+this.influence, 0, Math.PI*2);
                ctx.closePath();
                ctx.stroke();
            }
        }
    }
}

// Potential controller class, navigates based on the potential field
class PotentialController {
    // construct a new potential field
    constructor(robots,path,map) {
        this.path = path;
        this.robots = robots;
        this.dynamic_fields = [];
        this.map = map;

        this.static_field = new StaticPotentialField();
        this.gradient = 1.5;
        this.distance_from_path_factor = 0.4;

        this.heading_weighting = 15;
        this.velocity_weighting = 10;

        this.waypoints = [];
        for(let i = 0; i < this.robots.length; i++) {
            this.waypoints.push({x: 0, y: 0});
            this.dynamic_fields.push(new DynamicPotentialField(this.static_field, this.robots[i], this.robots));
        }

        this.gradient_search_radius = 20;

        this.break_distance = 40;
    }

    // control the robots based on the potential fields
    update() {
        for(let i = 0; i < this.robots.length; i++) {
            let robot = this.robots[i];

            // update dynamic field based on the robots positions
            let dynamic_field = this.dynamic_fields[i];
            dynamic_field.update();

            // get position and waypoint based on gradient
            let rx = robot.x;
            let ry = robot.y;
            let angle = this.normalizeAngle(robot.theta);

            let p = this.findLowestGradient(rx, ry, dynamic_field);
            let px = p.x; let py = p.y;

            // calculate direction theta
            let phi = Math.atan2(py-ry,px-rx); // angle from robot position to waypoint position
            let distance = Math.sqrt(Math.pow(px-rx,2) + Math.pow(py-ry,2)); // distance from robot to waypoint
            
            let alpha = this.normalizeAngle(phi - angle);

            // update waypoint for displaying purpose
            this.waypoints[i].x = px;
            this.waypoints[i].y = py;

            // change velocity based on the distance that the robots are to eachother
            let lowest_distance = 3000;
            let lowest_id = 0;
            for(let j = 0; j < this.robots.length; j++) {
                if(this.robots[j]!=robot) {
                    let other_robot = this.robots[j];
                    let distance = Math.sqrt(Math.pow(robot.x - other_robot.x, 2) + Math.pow(robot.y - other_robot.y,2));
                    if(distance < lowest_distance) {
                        //alert('Lowest: ' + distance);
                        lowest_distance = distance;
                        lowest_id = j;
                    }
                }
            }

            let break_factor = 0;
            if(lowest_distance < this.break_distance)
                break_factor = 0.6;

            // implement control
            robot.setVelocity(distance*this.velocity_weighting*(1-break_factor));
            robot.setOmega(alpha*this.heading_weighting);

            if(robot.holonomic) {
                robot.move(px, py, this.map, this.robots);
            }
        }
    }

    // find lowest gradient on static and optinal dynamic field
    findLowestGradient(x, y, dynamic_field) {
        let lowest_x = 0;
        let lowest_y = 0;
        let lowest = this.static_field.default;

        let r = this.gradient_search_radius;

        for(let j = -r; j <= r; j++) {
            for(let i = -r; i <= r; i++) {
                let value = this.static_field.default;
                if(dynamic_field == null)
                    value = Math.round(this.static_field.get(x-i, y-j));
                else
                    value = Math.round(this.static_field.get(x-i, y-j) + dynamic_field.get(x-i, y-j));

                if(lowest > value) {
                    lowest_x = x-i;
                    lowest_y = y-j;
                    lowest = value;
                }
            }
        }

        return {x:lowest_x, y:lowest_y};
    }


    // generate static potential 
    generateStaticPotentialField(path) {
        this.static_field.reset();
        this.path = path;

        // goal position
        let gx = this.path[this.path.length-1].x;
        let gy = this.path[this.path.length-1].y;
        
        // generate all the nodes and the corresponding costs from the path
        let nodes = [];
        let cost = 1;
        for(let i = this.path.length-1; i > 0; i--) {
            nodes.push({x:this.path[i].x, y:this.path[i].y, cost: cost})
            cost += this.gradient*this.distance(this.path[i].x, this.path[i].y, this.path[i-1].x, this.path[i-1].y);
        }

        // discretize path into points
        let path_points = [];
        for(let i = 0; i < nodes.length-1; i++) {
            let d = this.distance(nodes[i].x, nodes[i].y, nodes[i+1].x, nodes[i+1].y);
            let vx = (nodes[i+1].x - nodes[i].x)/d;
            let vy = (nodes[i+1].y - nodes[i].y)/d;
            let vcost = (nodes[i+1].cost - nodes[i].cost)/d;

            for(let j = 0; j < d; j++) {
                path_points.push({x: nodes[i].x + vx*j, y: nodes[i].y + vy*j, cost: nodes[i].cost + vcost*j});
            }
        }

        // generate static potential field based on distance from path points
        for(let y = 0; y < this.static_field.height; y++) {
            for(let x = 0; x < this.static_field.width; x++) {
                // find closest path point
                let nearest_distance = 1000;
                let nearest_id = 0;
                for(let i = 0; i < path_points.length; i++) {
                    let d = this.distance(x, y, path_points[i].x, path_points[i].y);
                    if(nearest_distance > d) {
                        nearest_distance = d;
                        nearest_id = i;
                    }
                }

                // set static field
                let path_point = path_points[nearest_id];
                let d = this.distance(x, y, path_point.x, path_point.y);
                this.static_field.set_min(x, y, path_point.cost + this.distance_from_path_factor*Math.pow(d, 2));
            }
        }

        // make the entire goal low
        for(let j = -25; j < 25; j++) {
            for(let i = -25; i < 25; i++) {
                let d = Math.sqrt(Math.pow(i, 2) + Math.pow(j, 2));
                let x = gx+i; let y = gy+j;
                this.static_field.set(x, y, Math.max(1+this.gradient*d,2));
            }
        }

        // set minimum at goal to allow formation
        this.static_field.set(gx - 20, gy + 20, 1);
        this.static_field.set(gx + 20, gy + 20, 1);
        this.static_field.set(gx, gy - 20, 1);

        // initialize static potential field based on the obstacles in the map
        this.setPotentialForAllObstacles();
    }

    // calculate distance between two positions
    distance(x1, y1, x2, y2) {
        return Math.sqrt(Math.pow(x1- x2, 2) + Math.pow(y1 - y2, 2));
    }

    // create a potential field around all the obstacles in the map
    setPotentialForAllObstacles() {
        for (var n = 0; n < this.map.obstacles.length; n++) {
            let obstacle = this.map.obstacles[n];
            switch (obstacle.type) {
                case 0: // square
                    this.static_field.setRepulsionRectangle(obstacle.x, obstacle.y, obstacle.w, obstacle.h, this.map.pathfinding_offset);
                    break;
                case 1: // circle
                    this.static_field.setRepulsionCircle(obstacle.x, obstacle.y, obstacle.r, this.map.pathfinding_offset);
                    break;
            }
        }
    }

    // draw waypoints that the robots are currently navigating towards
    drawWaypoint(ctx) {
        for(let i = 0; i < this.waypoints.length; i++) {
            let robot = this.robots[i];

            let rx = robot.x;
            let ry = robot.y;

            let waypoint = this.waypoints[i]; 
            let wx = waypoint.x;
            let wy = waypoint.y;

            ctx.strokeStyle = '#AA00BB';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(rx, ry);
            ctx.lineTo(wx, wy);
            ctx.stroke();
            ctx.closePath();
        }

        for(let i = 0; i < this.dynamic_fields.length; i++) {
            this.dynamic_fields[i].drawFast(ctx);
        }
    }

    // normalize the angles, returns angles between -PI and PI
    normalizeAngle(angle) {
        let fangle = (angle + Math.PI)%(2*Math.PI) - Math.PI;
        if(fangle < -Math.PI) {
            fangle = Math.PI + fangle%(Math.PI);
        }
        return fangle;
    }
}