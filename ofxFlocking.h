/*
 *  Flock.h
 *
 *  Created by Jeffrey Crouse on 3/30/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Modified by Rick Companje
 */


#pragma once

#include "ofMain.h"
#include "Boid.h"

class ofxFlocking {
public:
	void update(int hold);
	void draw();
	void addBoid();
	void addBoid(int x, int y);
	void killAll();
	
	vector<Boid> boids;
};
