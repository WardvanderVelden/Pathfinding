// Obstacle class, defines obstacles and collision detection algorithm for maps
class Obstacle {
	constructor() {
		this.x = 0;
		this.y = 0;
		this.w = 0

		this.color = '#0000FF';
	}

	// initialize circular obstacle
	circle(x,y,r) {
		this.type = 1;

		this.x = x;
		this.y = y;
		this.r = r;
	}

	// initialize rectangular obstacle
	rectangle(x,y,w,h) {
		this.type = 0;

		this.x = x;
		this.y = y;
		this.w = w;
		this.h = h;
	}

	// define random rectangle
	randomRectangle(minw, maxw, minh, maxh) {
		let x = Math.round(Math.random()*width);
		let y = Math.round(Math.random()*height);
		let w = Math.round(Math.random()*(maxw-minw)+minw);
		let h = Math.round(Math.random()*(maxh-minh)+minh);

		this.rectangle(x,y,w,h);
	}

	// define random circle
	randomCircle(minr,maxr) {
		let x = Math.round(Math.random()*width);
		let y = Math.round(Math.random()*height);
		let r = Math.round(Math.random()*(maxr-minr)+minr);

		this.circle(x,y,r);
	}

	// get pathfinding obstacle
	pathfindingObstacle(offset) {
		let obstacle = new Obstacle();
		switch(this.type) {
			case 0:
				obstacle.rectangle(this.x-offset, this.y-offset, this.w+2*offset, this.h+2*offset);
				break;
			case 1:
				obstacle.circle(this.x, this.y, this.r+offset);
				break;
		}
		return obstacle;
	}

	// check for collision with point
	pointCollision(px,py) {
		switch(this.type) {
			case 0:
				if(px > this.x && px < this.x+this.w && py > this.y && py < this.y+this.h) return true;
				break;
			case 1:
				var dist = Math.sqrt(Math.pow(px-this.x,2)+Math.pow(py-this.y,2))
				if(dist < this.r) return true;
				break;
		}
	}

	// check for collision with rectangle
	rectangularCollision(px,py,pw,ph) {
		switch(this.type) {
			case 0:
				if(this.x < px+pw && this.x+this.w > px && this.y < py+ph && this.y + this.h > py) return true;
				break;
			case 1:
				var dist = Math.sqrt(Math.pow(px-this.x,2)+Math.pow(py-this.y,2));
				dist = Math.min(dist, Math.sqrt(Math.pow(px+pw-this.x,2)+Math.pow(py-this.y,2)))
				dist = Math.min(dist, Math.sqrt(Math.pow(px-this.x,2)+Math.pow(py+ph-this.y,2)))
				dist = Math.min(dist, Math.sqrt(Math.pow(px+pw-this.x,2)+Math.pow(py+ph-this.y,2)))
				if(dist < this.r) return true;
				return false;
				break;
		}
		return false;
	}

	// check for collision with another obstacle
	obstacleCollision(obstacle) {
		switch(obstacle.type) {
			case 0:
				return this.rectangularCollision(obstacle.x, obstacle.y, obstacle.w, obstacle.y);
			case 1:
				return this.rectangularCollision(obstacle.x-obstacle.r, obstacle.y-obstacle.r, obstacle.r*2, obstacle.r*2);
		}	
	}

	// draw the obstacle
	draw(ctx, c) {
		if(c == null) c = this.color;

		switch(this.type) {
			case 0:
				ctx.fillStyle = c;
				ctx.fillRect(this.x, this.y, this.w, this.h);
				break;
			case 1:
				ctx.fillStyle = c;
				ctx.beginPath();
				ctx.arc(this.x, this.y, this.r, 0, 2 * Math.PI)
				ctx.fill();
				ctx.closePath();
				break;
		}
	}
}


// Map class, contains the obstacles
class Map {
	constructor() {
		this.obstacles = [];
		this.pathfinding_obstacles = [];

		this.pathfinding_offset = 20;

		this.random_obstacles_can_collide = false;
	}

	// add rectangular obstacle and larger pathfinding obstacle
	addRectangle(x,y,w,h) {
		// map obstacle
		var obstacle = new Obstacle();
		obstacle.rectangle(x, y, w, h);
		this.obstacles.push(obstacle);
		this.pathfinding_obstacles.push(obstacle.pathfindingObstacle(this.pathfinding_offset));
	}

	// add circular obstacle and larger pathfinding obstacle
	addCircle(x,y,r) {
		// map obstacle
		var obstacle = new Obstacle();
		obstacle.circle(x, y, r);
		this.obstacles.push(obstacle);
		this.pathfinding_obstacles.push(obstacle.pathfindingObstacle(this.pathfinding_offset));
	}

	// add an obstacle with a pathfinding obstacle to the map
	addObstacleWithPathfinding(obstacle) {
		this.obstacles.push(obstacle);
		this.pathfinding_obstacles.push(obstacle.pathfindingObstacle(this.pathfinding_offset));
	}

	// generate a random map based on the safe areas
	randomMap(amount) {
		while(this.obstacles.length < amount) {
			var type = Math.round(Math.random());
			var obstacle =  new Obstacle();

			switch(type) {
				case 0:
					obstacle.randomRectangle(25,150,25,150);
					break;
				case 1:
					obstacle.randomCircle(25,90);
					break;
			}
			if(this.random_obstacles_can_collide) this.addObstacleWithPathfinding(obstacle);
			else {
				var collides = false;
				for(var i = 0; i < this.obstacles.length; i++) {
					if(this.obstacles[i].obstacleCollision(obstacle)) {
						collides = true;
						break;
					}
				}
				if(!collides) {
					this.addObstacleWithPathfinding(obstacle);
				}
			}
		}
	}

	// define map
	defineMap(map_number) {
		switch(map_number) {
			case 0:
				this.addRectangle(300, 0, 25, 125);
				this.addRectangle(0, 200, 500, 25);
				this.addRectangle(500, 200, 25, 300);
				this.addRectangle(525, 350, 150, 25);
				this.addRectangle(675, 350, 25, 300);
				this.addRectangle(250, 350, 25, 350);
				this.addRectangle(275,500,250,25);
				
				this.addCircle(500,650,60);
				this.addRectangle(440,650,120,150);

				this.addCircle(700,150,50);
				this.addRectangle(650,0,150,150);
				this.addRectangle(700,150,100,50);

				this.addCircle(200,400,50);
				this.addRectangle(200,350,75,100);

				this.addCircle(50,600,100);
				this.addRectangle(0,500,50,200);

				this.addCircle(500,200,75);
				break;
			case 1:
				this.addCircle(width/2, height/2, 100);
				this.addCircle(width/2+100, height/2, 100);
				this.addRectangle(width/2,height/2-100,100,200);
				break;

			case 2:
				this.randomMap(30);
				break;
			case 3:
				this.addRectangle(0, 300, 400, 25);
				this.addRectangle(375, 100, 25, 200);
				this.addRectangle(100, 100, 150, 100);
				this.addRectangle(525, 125, 250, 150);
				this.addRectangle(575, 350, 25, 150);
				this.addCircle(200, 500, 75);
				this.addRectangle(200,425,175,150);
				this.addRectangle(500, 500, 300, 25);
				this.addRectangle(700, 525, 25, 200);
				this.addRectangle(450, 600, 150, 75);
				break;
		}
	}

	// check if point collides with the map
	checkPointCollision(x, y) {
		for(var i = 0; i < this.obstacles.length; i++) {
			if(this.obstacles[i].pointCollision(x,y)) return true;
		}
		if(x < 0 || x > width || y < 0 || y > height) return true;
		return false;
	}

	// check if point collides with pathfinding map
	checkPointCollisionForPathfinding(x, y) {
		for(var i = 0; i < this.obstacles.length; i++) {
			if(this.pathfinding_obstacles[i].pointCollision(x,y)) return true;
		}
		if(x < this.pathfinding_offset || x > width-this.pathfinding_offset || y < this.pathfinding_offset || y > height-this.pathfinding_offset) return true;
		return false;
	}

	// draw the map
	draw(ctx) {
		let pf_c = '#000066';
		for(var i = 0; i < this.pathfinding_obstacles.length; i++) {
			this.pathfinding_obstacles[i].draw(ctx, pf_c);
		}

		// edge boundaries
		ctx.fillStyle = pf_c;
		ctx.fillRect(0,0,this.pathfinding_offset,height);
		ctx.fillRect(0,0,width,this.pathfinding_offset);
		ctx.fillRect(0,height-this.pathfinding_offset,width,this.pathfinding_offset);
		ctx.fillRect(width-this.pathfinding_offset,0,width-this.pathfinding_offset,height);

		for(var i = 0; i < this.obstacles.length; i++) {
			this.obstacles[i].draw(ctx);
		}
	}
}