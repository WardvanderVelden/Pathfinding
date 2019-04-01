class Robot {
	constructor(x,y,dt) {
		this.x = x;
		this.y = y;

		this.v = 10;
		this.a = 0;

		this.max_v = 250;
		this.max_a = 10;

		this.theta = 0.01;
		this.omega = 0.01;
		this.alpha = 0.01;

		this.max_omega = 200;
		this.max_alpha = 20;

		this.radius = 10;

		this.dt = dt;

		this.trace = [];

		this.no_accelerations = false;
		this.holonomic = false;
	}

	// integrate the state of the robot
	update(map, robots) {
		if(!this.holonomic) {
			var nx = this.x + Math.cos(this.theta)*this.v*this.dt;
			var ny = this.y + Math.sin(this.theta)*this.v*this.dt;

			this.move(nx, ny, map, robots);
			
			this.v += this.a*this.dt;

			this.omega += this.alpha*this.dt;
			this.theta += this.omega*this.dt;

			this.theta = this.theta%(2*Math.PI);
		}	

		this.trace.push({x:this.x,y:this.y});
	}

	notMoving() {
		if(Math.round(this.v) == 0 && Math.round(this.a) == 0 && Math.round(this.alpha) == 0 && Math.round(this.omega) == 0) {
			/*console('Robot not moving - final state:');
			console('X: ' + this.x);
			console('Y: ' + this.y);
			console('Velocity: ' + this.v);
			console('Acceleration: ' + this.a);
			console('Theta: ' + this.theta);
			console('Omega: ' + this.omega);
			console('Alpha: ' + this.alpha);*/
			return true;
		}
		return false;
	}

	isInGoal(pf) {
		if(this.x > pf.gx && this.y > pf.gy && this.x < (pf.gx + pf.gw) && this.y < (pf.gy + pf.gh))
			return true;
		return false;
	}

	// set goal omega and update angular acceleration based on that
	setOmega(omega) {
		if(!this.no_accelerations) {
			var goal = 0;
			if(omega >= 0) {
				goal = Math.min(omega, this.max_omega)
			} else {
				goal = Math.max(omega, -this.max_omega);
			}

			var diff = goal - this.omega;
			this.alpha = diff*this.max_alpha;
		} else {
			this.omega = omega;
			this.alpha = 0;
		}	
	}

	// set goal velocity and update acceleration based on that
	setVelocity(v) {
		if(!this.no_accelerations) {
			var goal = 0;
			if(v >= 0) {
				goal = Math.min(v, this.max_v)
			} else {
				goal = Math.max(v, -this.max_v);
			}

			var diff = goal - this.v;
			this.a = diff*this.max_a;
		} else {
			this.v = v;
			this.a = 0;
		}
		
	}

	// check if new position is a legal one
	move(nx, ny, map, robots) {
		var canMove = true;

		// world boundary check
		if(nx - this.radius < 0 || nx + this.radius > width || ny - this.radius < 0 || ny + this.radius > height) 
			canMove = false;
		
		// collision check with map
		for(var i = 0; i < map.obstacles.length; i++) {
			var obstacle = map.obstacles[i];
			if(obstacle.rectangularCollision(nx-this.radius,ny - this.radius, 2*this.radius, 2*this.radius)) {
				canMove = false;
				break;
			}
		}

		// collision check with other robots
		if(robots != null) {
			for(let i = 0; i < robots.length; i++) {
				let robot = robots[i];
				if(robot != this) {
					let d = Math.sqrt(Math.pow(this.x - robot.x, 2) + Math.pow(this.y - robot.y, 2));
					if(d < (this.radius+robot.radius)) {
						canMove = false;
						break;
					}
				}
			}
		}

		// update move
		if(canMove) {
			this.x = nx;
			this.y = ny;
		} else {
			this.v = 0;
			this.a = 0;

			this.omega = 0;
			this.alpha = 0;
		}
	}

	// reset
	reset() {
		this.trace = [];
		
		this.v = 10;
		this.a = 0;

		this.theta = 0;
		this.omega = 0;
		this.alpha = 0;
	}

	// draw the robot
	draw(ctx) {
		ctx.fillStyle = '#FF0000';
		ctx.beginPath();
		ctx.arc(this.x, this.y, this.radius, 0, 2 * Math.PI)
		ctx.fill();
		ctx.closePath();

		ctx.strokeStyle = '#00FFFF';
		ctx.lineWidth = 2;
		ctx.beginPath();
		ctx.moveTo(this.x, this.y);
		ctx.lineTo(this.x + Math.cos(this.theta)*this.radius*1.5, this.y + Math.sin(this.theta)*this.radius*1.5);
		ctx.stroke();
		ctx.closePath();
	}

	// draw the trace of the robot
	drawTrace(ctx, color) {
		if(this.trace.length === 0) return;
		if(color != null)
			ctx.strokeStyle = color;
		else
			ctx.strokeStyle = '#FFFFFF';
		
		ctx.lineWidth = 1;
		ctx.beginPath();
		ctx.moveTo(this.trace[0].x,this.trace[0].y);
		for(var i = 0; i < this.trace.length; i++) {
			ctx.lineTo(this.trace[i].x,this.trace[i].y);
		}
		ctx.stroke();
		ctx.closePath();
	}
}