// Vector class
class Vector {
	constructor(x,y) {
		this.x = x;
		this.y = y;
	}

	norm() {
		return Math.sqrt(Math.pow(this.x,2)+Math.pow(this.y,2));
	}

	distanceToPoint(x,y) { 
		return Math.sqrt(Math.pow(this.x-x,2)+Math.pow(this.y-y,2));
	}

	distanceToVector(pos) {
		return Math.sqrt(Math.pow(this.x-pos.x,2)+Math.pow(this.y-pos.y,2));
	}
}


// Node class to define nodes
class Node {
	constructor(from, x, y, id, cost) {
		this.from = from; // from node or nodes
		this.to = null; // to node or nodes

		this.id = id;
		this.pos = new Vector(x, y);
		this.cost = cost;
	}

	draw(ctx, highest_cost) {
		if(this.from == null) return;
		if(highest_cost == null) highest_cost = diagonal;

		ctx.fillStyle = this.getColor(highest_cost);
		ctx.fillRect(this.pos.x-2,this.pos.y-2,4,4);

		ctx.lineWidth = 1;
		if(Array.isArray(this.from)) {
			for(var i = 0; i < this.from.length; i++) {
				let node = this.from[i];

				ctx.beginPath();
				ctx.moveTo(node.pos.x, node.pos.y);
				ctx.lineTo(this.pos.x, this.pos.y);
				ctx.stroke();
				ctx.closePath();
			}
		} else {
			let gradient = ctx.createLinearGradient(this.from.pos.x, this.from.pos.y, this.pos.x, this.pos.y);
			gradient.addColorStop('0', this.from.getColor(highest_cost));
			gradient.addColorStop('1', this.getColor(highest_cost));

			ctx.strokeStyle = gradient;

			ctx.beginPath();
			ctx.moveTo(this.from.pos.x, this.from.pos.y);
			ctx.lineTo(this.pos.x, this.pos.y);
			ctx.stroke();
			ctx.closePath();
		}	
	}

	getColor(highest_cost) {
		if(highest_cost == null) highest_cost = diagonal;

		// scale for gradient calculation
		let scaled_cost = Math.min(this.cost,highest_cost)/(highest_cost)*2;
		let b = 0; let g = 0; let r = 0;

		if(scaled_cost < 1) {
			b = (1-scaled_cost)*255;
			g = scaled_cost*255;
		} else {
			g = (1 - (scaled_cost - 1))*255;
			r = (scaled_cost - 1)*255;
		}

		return 'rgb('+r+','+g+','+b+')';
	}
}


// RRT path finding algorithm
class RRT {
	constructor(map) {
		this.map = map; // reference to map object
		this.nodes = [];

		this.max_nodes = 2000;
		this.max_waypoints = 500;

		this.path = [];
		this.path_points = [];

		this.hasPath = false;

		this.draw = true;

		this.discretization_step = 25;
		this.discretization_step_optimization = 3;

		this.expansion_rate = 10;
		this.expansion = this.expansion_rate;
	}

	// reset path finding object
	reset() {
		this.path = [];
		this.path_points = [];
		this.nodes = [];

		this.hasPath = false;

		this.expansion = this.expansion_rate;
	}

	// get vector within expansion circle
	getRandomVectorWithinExpansion() {
		let has_vector = false;
		let x = 0;
		let y = 0;
		while(!has_vector) {
			x = Math.round(Math.random() * width);
			y = Math.round(Math.random() * height);

			let distance = this.start.distanceToPoint(x, y);
			if(distance < this.expansion) {
				has_vector = true;
				this.expansion += this.expansion_rate;
			}
		}
		return new Vector(x, y);
	}

	// run the RRT algorithm with radial expansion from start 
	getGraph(ctx) {
		this.nodes = [];

		// set initial node at start
		this.addNode(null, this.sx, this.sy);

		// iterate until max nodes is reached
		while(!this.hasPath && this.nodes.length < this.max_nodes) {
			var pos = this.getRandomVectorWithinExpansion();
			
			// find closest collision free straight-lined-path node
			var lowest_node_id = null;
			var lowest_distance = 10000;
			for(var i = 0; i < this.nodes.length; i++) {
				var node = this.nodes[i];

				// set lowest distance and node if distance is lower
				var dist = this.feasibleDistancePathToNode(node, pos.x, pos.y);
				if(dist < lowest_distance && dist > 0) {
					lowest_node_id = node.id;
					lowest_distance = dist;
				}
			}

			// add node and path if a feasible path exist and is lowest distance
			if(lowest_node_id != null) {
				this.addNode(this.getNode(lowest_node_id), pos.x, pos.y);

				// draw for debug purposes
				if(this.draw) this.nodes[this.nodes.length-1].draw(ctx);

				// check if graph has reached the goal
				if(pos.x > this.gx && pos.y > this.gy && pos.x < this.gx + this.gw && pos.y < this.gy + this.gh) {
					this.getPathFromGraph();
				}
			}
		}
		if(!this.hasPath) {
			console('No path found within node limit');
		}
	}

	// run optimization iterations
	optimize(runs) {
		if(this.path.length === 0) return;

		console('Non optimal - amount of waypoints: ' + this.path.length);
		for(var i = 0; i < runs; i++) {
			
			console((i+1) + '# optimization run');
			this.optimizePath();
			console('&emsp;Amount of waypoints: ' + this.path.length);
		}
	}

	// optimize entire RRT path, once again a homebrew iteration process but should be optimal
	optimizePath() {
		if(this.path.length === 0) return;
		this.getPathPoints();

		var i = 0;
		var last_visible_index = 0;
		while(i != this.path_points.length-1) {
			var ox = this.path_points[i].pos.x;
			var oy = this.path_points[i].pos.y;

			// check all points from this optimization point
			var ever_invisible = false; var was_visible = false;
			for(var j = i; j < this.path_points.length-1; j++) {
				var cx = this.path_points[j].pos.x;
				var cy = this.path_points[j].pos.y;

				if(!this.checkLineCollision(ox,oy,cx,cy,this.discretization_step_optimization)) was_visible = true;
				else if(was_visible) {
					was_visible = false;
					ever_invisible = true;
					if(last_visible_index != j-1) {
						last_visible_index = j-1;
						// remove waypoints that are not on this path and add new waypoint if the distance is larger than discretization step
						var start_new_waypoint = this.path_points[i];
						var end_new_waypoint = this.path_points[j];

						if(start_new_waypoint.pos.distanceToVector(end_new_waypoint.pos) > this.discretization_step) {
							this.path.splice(start_new_waypoint.to_waypoint, end_new_waypoint.to_waypoint - start_new_waypoint.to_waypoint, new Vector(start_new_waypoint.pos.x, start_new_waypoint.pos.y), new Vector(end_new_waypoint.pos.x, end_new_waypoint.pos.y));
							this.getPathPoints();
						}
					}
				}
			}
			if(!ever_invisible) { // in this case we can see the end and should remove all the junk in between
				var start_new_waypoint = this.path_points[i];
				this.path.splice(start_new_waypoint.to_waypoint, this.path.length - 1 - start_new_waypoint.to_waypoint, new Vector(start_new_waypoint.pos.x, start_new_waypoint.pos.y), new Vector(this.gx+this.gw/2, this.gy+this.gh/2));
				this.getPathPoints();
				i = this.path_points.length-2;

			}

			// set limit to optimization
			if(this.path.length > this.max_waypoints) {
				console("&emsp;Waypoint limit in optimization exceeded");
				return;
			}

			i++;
		}
	}

	// calculate distance from a node to a position and check feasibility
	feasibleDistancePathToNode(node, x, y) {
		var dist = node.pos.distanceToPoint(x, y);
		if(!this.checkLineCollision(node.pos.x, node.pos.y, x, y)) return dist;
		else return false;
	}

	// check line collision on with respect to the map
	checkLineCollision(ax,ay,bx,by, disc_step) {
		if(disc_step == null) disc_step = this.discretization_step;

		var distance = Math.sqrt(Math.pow(ax-bx,2)+Math.pow(ay-by,2));
		var segments = distance/disc_step;

		var vx = (ax - bx)/segments;
		var vy = (ay - by)/segments;

		for(var i = 0; i <= segments; i++) {
			var cx = bx + vx*i;
			var cy = by + vy*i;

			if(this.map.checkPointCollisionForPathfinding(cx, cy)) return true;
		}
		return false;
	}

	// reverse order to get the path from the graph
	getPathFromGraph() {
		this.path.push(new Vector(this.gx+this.gw/2,this.gy+this.gh/2));

		var node = this.nodes[this.nodes.length-1].from;
		while(node != this.nodes[0]) {
			this.path.push(new Vector(node.pos.x, node.pos.y));
			node = node.from;
		}

		// reverse order of path
		this.path = this.path.reverse();

		this.hasPath = true;
		console('Amount of nodes: ' + this.nodes.length);
	}

	// get node based on id
	getNode(id) {
		for(var i = 0; i < this.nodes.length; i++) {
			var node = this.nodes[i];
			if(node.id === id) return node;
		}
	}

	// get path points using the defined discritization step
	getPathPoints() {
		if(this.path.length === 0) return;
		this.path_points = [];

		// get initial previous waypoint x
		var px = this.sx;
		var py = this.sy;
		for(var i = 0; i < this.path.length; i++) {
			var waypoint = this.path[i];
			var dist = waypoint.distanceToPoint(px, py);

			var segments = dist/this.discretization_step_optimization;
			var vx = (waypoint.x - px)/segments;
			var vy = (waypoint.y - py)/segments;

			for(var j = 0; j < segments; j++) {
				var cx = px + vx*j;
				var cy = py + vy*j;
				this.path_points.push({pos: new Vector(cx, cy),to_waypoint:i});
			}

			px = waypoint.x;
			py = waypoint.y;
		}
	}

	get highest_cost() {
		let cost = 0;
		for(var i = 0; i < this.nodes.length; i++)
			if(this.nodes[i].cost > cost) cost = this.nodes[i].cost;
		return cost;
	}

	// set start of the RRT algorithm
	setStart(x, y) {
		this.sx = x;
		this.sy = y;
		this.start = new Vector(x, y);
	}

	// set goal of the RRT algorithm
	setGoal(x,y,w,h) {
		this.gx = x;
		this.gy = y;
		this.gw = w;
		this.gh = h;
	}

	// add node to list of nodes
	addNode(from, x, y) {
		var id = this.nodes.length;
		let vector = new Vector(x, y);
		let cost = 0;
		if(from == null) cost = 0;
		else cost = from.cost + vector.distanceToVector(from.pos);
		this.nodes.push(new Node(from, x, y, id, cost));
	}

	// draw the path that has been found
	drawPath(ctx, c) {
		if(c == NaN) ctx.strokeStyle = '#AA0000';
		else ctx.strokeStyle = c;
		ctx.lineWidth = 1.25;
		ctx.beginPath();
		ctx.moveTo(this.sx,this.sy);
		for(var i = 0; i < this.path.length; i++) {
			ctx.lineTo(this.path[i].x,this.path[i].y);
		}
		ctx.stroke();
		ctx.closePath();

		let s = 4;
		ctx.fillStyle = "#990000";
		for(var i = 0; i < this.path.length; i++) {
			let x = this.path[i].x;
			let y = this.path[i].y;

			ctx.fillRect(x-s,y,s*2,1);
			ctx.fillRect(x,y-s,1,s*2);
		}
	}

	// draw the path points
	drawPathPoints(ctx) {
		ctx.fillStyle = '#AA00AA';
		let size = 2;
		for(var i = 0; i < this.path_points.length; i++) {
			ctx.fillRect(this.path_points[i].x-size/2, this.path_points[i].y-size/2,size,size);
		}
	}

	// draw goal
	drawGoal(ctx) {
		ctx.fillStyle = '#009990';
		ctx.fillRect(this.gx, this.gy, this.gw, this.gh);
	}

	// draw graph
	drawGraph(ctx) {
		for(var i = 0; i < this.nodes.length; i++) {
			this.nodes[i].draw(ctx, this.highest_cost);
		}
	}
}


// RRT smart path finding, not quite RRT star but something resembling it and extremely fast
class RRTsmart extends RRT {
	constructor(map) { 
		super(map);

		this.max_nodes = 1250;
		this.expansion_rate = 5;

		this.search_for_optimal = true;
		this.goal_reached_nodes = [];
	}

	reset() {
		super.reset();
		this.goal_reached_nodes = [];
	}

	// run the RRT* algorithm with radial expansion from start position 
	getGraph() {
		this.nodes = [];

		// set initial node at start
		this.addNode(null, this.sx, this.sy); 

		// iterate until max nodes is reached
		while(!this.hasPath && this.nodes.length < this.max_nodes) {
			var pos = this.getRandomVectorWithinExpansion(); // get a position vector within the expansion circle

			// go through all nodes and see if they are connectable, if connectable, add the to the tree the one with the lowest cost (half RRT*)
			var lowest_cost_node = null;
			var lowest_cost = 10000;
			for(var i = 0; i < this.nodes.length; i++) {
				var node = this.nodes[i];

				// find lowest cost connecting node
				var distance = this.feasibleDistancePathToNode(node, pos.x, pos.y);
				if(distance != false) {
					var cost = node.cost + distance;
					if(cost < lowest_cost) {
						lowest_cost = cost;
						lowest_cost_node = node;
					}
				}
			}
			
			// add node
			if(lowest_cost_node != null) {
				this.addNode(lowest_cost_node, pos.x, pos.y);
				var last_node = this.nodes[this.nodes.length-1];

				// check if graph has reached the goal
				if(pos.x > this.gx && pos.y > this.gy && pos.x < this.gx + this.gw && pos.y < this.gy + this.gh) {
					if(this.search_for_optimal) this.goal_reached_nodes.push(last_node);
					else this.getPathFromGraph();
				}
			}
		}
		if(this.search_for_optimal) this.getPathFromGraphReachedNodes();
	}

	// get path from graph function similar to the function in ordinary RRT but this goes through all goal reached nodes to get the cheapest path
	getPathFromGraphReachedNodes() {
		if(this.goal_reached_nodes.length === 0) 
			console("No path found using this graph!");
		else {
			this.path.push(new Vector(this.gx+this.gw/2,this.gy+this.gh/2));

			let lowest_cost = 10000;
			let lowest_node = null;
			for(var i = 0; i < this.goal_reached_nodes.length; i++) {
			 	let node = this.goal_reached_nodes[i];

			 	if(node.cost < lowest_cost) {
			 		lowest_cost = node.cost;
			 		lowest_node = node;
			 		
			 	}
			}
			var node = lowest_node.from;
			while(node != this.nodes[0]) {
				this.path.push(new Vector(node.pos.x, node.pos.y));
				node = node.from;
			}

			// reverse order of path
			this.path = this.path.reverse();

			this.hasPath = true;
			console('Amount of nodes: ' + this.nodes.length);
		}
	}
}

// RRT star path finding
class RRTstar extends RRT {
	constructor(map) { 
		super(map);

		this.max_nodes = 750;
		this.expansion_rate = 5;

		this.search_for_optimal = true;
		this.goal_reached_nodes = [];

		this.min_reroute_distance = 50;
		this.reroute_factor = this.max_nodes*this.min_reroute_distance; // reroute_factor/number of nodes
	}

	reset() {
		super.reset();
		this.goal_reached_nodes = [];
	}

	// run the RRT* algorithm with radial expansion from start position 
	getGraph() {
		this.nodes = [];

		// set initial node at start
		this.addNode(null, this.sx, this.sy); 

		// iterate until max nodes is reached
		while(!this.hasPath && this.nodes.length < this.max_nodes) {
			var pos = this.getRandomVectorWithinExpansion(); // get a position vector within the expansion circle

			// go through all nodes and see if they are connectable, if connectable, add the to the tree the one with the lowest cost (half RRT*)
			var lowest_cost_node = null;
			var lowest_cost = 10000;
			for(var i = 0; i < this.nodes.length; i++) {
				var node = this.nodes[i];

				// find lowest cost connecting node
				var distance = this.feasibleDistancePathToNode(node, pos.x, pos.y);
				if(distance != false) {
					var cost = node.cost + distance;
					if(cost < lowest_cost) {
						lowest_cost = cost;
						lowest_cost_node = node;
					}
				}
			}
			
			// add node
			if(lowest_cost_node != null) {
				this.addNode(lowest_cost_node, pos.x, pos.y);
				var last_node = this.nodes[this.nodes.length-1];

				// check if graph has reached the goal
				if(pos.x > this.gx && pos.y > this.gy && pos.x < this.gx + this.gw && pos.y < this.gy + this.gh) {
					if(this.search_for_optimal) this.goal_reached_nodes.push(last_node);
					else this.getPathFromGraph();
				}

				// reroute old nodes if new cost is more efficient (the star part)
				//let reroute_distance = this.min_reroute_distance;
				let reroute_distance = this.reroute_factor/this.nodes.length;
				for(var i = 0; i < this.nodes.length; i++) {
					var node_i = this.nodes[i];
					if(last_node.pos.distanceToVector(node_i) > reroute_distance) continue; // only check other points within the rerouting distance of the last point
					for(var j = 0; j < this.nodes.length; j++) {
						if(i === j) continue;
						var node_j = this.nodes[j];
						var distance = node_i.pos.distanceToVector(node_j.pos); 
						if(distance < reroute_distance) { // only check feasibility if other point is within rerouting distance
							if(this.feasibleDistancePathToNode(node_i, node_j.pos.x, node_j.pos.y) != false) {
								let alternative_cost = node_i.cost+distance;
								if(alternative_cost < node_j.cost) {
									node_j.from = node_i;
									node_j.cost = alternative_cost;
								}
							}
						}		
					}
				}
			}
		}
		if(this.search_for_optimal) this.getPathFromGraphReachedNodes();
	}

	// get path from graph function similar to the function in ordinary RRT but this goes through all goal reached nodes to get the cheapest path
	getPathFromGraphReachedNodes() {
		if(this.goal_reached_nodes.length === 0) 
			console("No path found using this graph!");
		else {
			this.path.push(new Vector(this.gx+this.gw/2,this.gy+this.gh/2));

			let lowest_cost = 10000;
			let lowest_node = null;
			for(var i = 0; i < this.goal_reached_nodes.length; i++) {
			 	let node = this.goal_reached_nodes[i];

			 	if(node.cost < lowest_cost) {
			 		lowest_cost = node.cost;
			 		lowest_node = node;
			 		
			 	}
			}
			var node = lowest_node.from;
			while(node != this.nodes[0]) {
				this.path.push(new Vector(node.pos.x, node.pos.y));
				node = node.from;
			}

			// reverse order of path
			this.path = this.path.reverse();

			this.hasPath = true;
			console('Amount of nodes: ' + this.nodes.length);
		}
	}
}