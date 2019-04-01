class Controller {
	constructor(robot, path) {
		this.robot = robot; // reference to controller
		this.path = path; // reference to path

		this.current_waypoint = 0;
		this.distance_tolerance = 50;

		this.heading_weighting = 8;
		this.velocity_weighting = 4;
	}

	// update control law and implement
	update() {
		// get relevant data
		var rx = this.robot.x;
		var ry = this.robot.y;
		var angle = this.normalizeAngle(this.robot.theta);

		var wx = this.path[this.current_waypoint].x;
		var wy = this.path[this.current_waypoint].y;

		var ex = this.path[this.path.length-1].x;
		var ey = this.path[this.path.length-1].y;

		// calculate direction theta
		var phi = Math.atan2(wy-ry,wx-rx); // angle from robot position to waypoint position
		var distance = Math.sqrt(Math.pow(wx-rx,2) + Math.pow(wy-ry,2)); // distance from robot to waypoint
		var distance_to_goal = Math.sqrt(Math.pow(ex-rx,2) + Math.pow(ey-ry,2));

		var alpha = this.normalizeAngle(phi - angle);

		// implement control
		this.robot.setVelocity(distance*this.velocity_weighting);
		this.robot.setOmega(alpha*this.heading_weighting);

		// go to the next waypoint if we are within the waypoint tolerance
		if(distance < this.distance_tolerance && (this.current_waypoint + 1) != this.path.length) {
			if((this.current_waypoint + 1) != this.path.length) this.current_waypoint++;
		}

		if(distance_to_goal < this.distance_tolerance) {
			this.robot.setVelocity(2*distance);
		}
	}

	reset() {
		this.current_waypoint = 0;
	}

	setPath(path) {
		this.path = path;
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