/*
 *  Boid.cpp
 *  boid
 *
 *  Created by Jeffrey Crouse on 3/29/10.
 *  Copyright 2010 Eyebeam. All rights reserved.
 *  Updated by Rick Companje
 *
 */

#include "Boid.h"



Boid::Boid() {
    loc.set(ofRandomWidth(),ofRandomHeight());
	vel.set(0,0);
	acc.set(0,0);
	predict.set(0,0);
	avg.set(0,0);
	pushLoc.set(0,0);
	
    r = 3.0;
    maxspeed = 0.61;
	maxforce = 0.93;
	neighbordist = 48.42;
	maxSteerForce = 0;
	desiredseparation = 0;
	//s = 0;
	//a = 0;
	c = 0;
	angle = 0;
	
	xNoise = 0;
	yNoise = 0;
	zNoise = 0;
	xNoiseInc = 0;
	yNoiseInc = 0;
	zNoiseInc = 0;
    pushVel = 0;
	
}

Boid::Boid(int x, int y) {
    loc.set(x,y);
	
	origin.set(x,y);
	
	vel.set(0.1,0);
	acc.set(0,0);
	
    r = 160.0;
	
	maxspeed = 0.61;
	maxforce = 0.93;
	neighbordist = 48.42;
	
	xNoise = 0;
	yNoise = 0;
	zNoise = 0;
	xNoiseInc = ofRandom(0.001, 0.007);
	yNoiseInc = ofRandom(0.01, 0.07);
	zNoiseInc = ofRandom(0.001, 0.007);
	
}

void Boid::setLoc(ofVec2f l){
    loc = l;

}

void Boid::updateValues(float _s , float _a , float _c,
						float _sF, float _sS, float _dS, float _sN){
	s = _s; //Separation
	a = _a; //Alignement
	c = _c; //Cohesion
	maxSteerForce = _sF;
	maxspeed = _sS;
	neighbordist = _sN;
	desiredseparation = _dS;

}



void Boid::push(float ms){
	ofVec2f h;
	
	psh = ms;
		//float a = ofMap(angle, -PI, PI, -1,1);
	
	predict *= (25*ms);
	ofVec2f predictLoc = loc + predict;

	ofLine(loc.x, loc.y, predictLoc.x, predictLoc.y);
		
	//pushLoc = predictLoc;
    
    // add push for flying in sequence
    // maybe start them farther off screen, and let them fly in during sequence?
    if (hold == 1) {
        loc = predictLoc;
    }else{
       // videoVel = mappedFlap*upMult;
         pushVel = psh * s;
    }
	
	
	ofSetColor(244, 0, 0);
	ofEllipse(loc.x, loc.y, ms*25, ms*25);
	ofSetColor(255, 255, 255,255);
	
}




// Method to update location
void Boid::update(vector<Boid> &boids, int h) {
	
	predict.set(vel);
	predict.normalize();
	hold = h;
	
	
	
	avgVel.push_back(vel);
	
	if(avgVel.size()> 20){
		avgVel.erase(avgVel.begin());
		
	}
	
	for(int i = 0; i < avgVel.size(); i ++){
		avg += avgVel[i];
	}
	
	avg /= avgVel.size();
		
	if(hold == 1){  //IF HOLD
			
	flock(boids);
		
	vel += acc;   // Update velocity
    vel.x = ofClamp(vel.x, -maxspeed, maxspeed);  // Limit speed
	vel.y = ofClamp(vel.y, -maxspeed, maxspeed);  // Limit speed
    loc += vel;
	
	
    acc = 0;  // Reset accelertion to 0 each cycle
		
	/*
	if (loc.x <  - r -100) loc.x = (ofGetWidth())+r +100  ;
    if (loc.y < - r  ) loc.y =	 (ofGetHeight())+r +100 ;

    if (loc.x > ((ofGetWidth()))+r+100) loc.x = -r-100 ;
    if (loc.y > (ofGetHeight())+r+100) loc.y =  -r ;
	*/
		
	}
	else
	{
        addNoise();
		vel.set(1,0);
        loc.x += pushVel;
        if (loc.x > origin.x) {  // pull back to original location
            pushVel -= c; // 0.007
        }
		//loc = origin;
		
	}
	
}


void Boid::seek(ofVec2f target) {

    acc += neighbordist*steer(target, false);
}

void Boid::avoid(ofVec2f target) {
    acc -= steer(target, false);
}

void Boid::arrive(ofVec2f target) {
    acc += steer(target, true);
}

// A method that calculates a steering vector towards a target
// Takes a second argument, if true, it slows down as it approaches the target

ofVec2f Boid::steer(ofVec2f target, bool slowdown) {
    ofVec2f steer;  // The steering vector
    ofVec2f desired = target - loc;  // A vector pointing from the location to the target
    
	float d = ofDist(target.x, target.y, loc.x, loc.y); // Distance from the target is the magnitude of the vector
	
	
	// If the distance is greater than 0, calc steering (otherwise return zero vector)
    if (d > 0) {
		
		desired /= d; // Normalize desired
		// Two options for desired vector magnitude (1 -- based on distance, 2 -- maxspeed)
		if ((slowdown) && (d < 100.0f)) {
			desired *= maxspeed * (d/100.0f); // This damping is somewhat arbitrary
		} else {
			desired *= maxspeed;
		}
		// Steering = Desired minus Velocity
		steer = desired - vel;
		
		
		//steer.x = ofMap(steer.x, 0, steer.x, -maxforce, +maxforce);
		//steer.y = ofMap(steer.y, 0, steer.y, -maxforce, +maxforce);
		
		steer.x = ofClamp(steer.x, -maxSteerForce, maxSteerForce); // Limit to maximum steering force
		steer.y = ofClamp(steer.y, -maxSteerForce, maxSteerForce); 
		
		
		
    }
    return steer;
}

ofVec3f Boid::addNoise(){
	
	//xNoise += xNoiseInc;
	yNoise += yNoiseInc;
	//zNoise += zNoiseInc;
	
	//noise.x = ofNoise(xNoise);
	noise.y = sin(yNoise);  // up an down
	
	return noise;
	
}

ofVec2f Boid::getPredictLoc(){
	ofVec2f p(predict);
	p *=75;
	p += loc;
	return p;

}

float Boid::getScale(){
	
	zNoise += zNoiseInc;
	float z = sin(zNoise);
	return z;
	

}

ofVec3f Boid::getLoc(){
	
	ofVec3f l;
	
	//float angle =(float)atan2(-avg.y, avg.x);
	float angle = (float)atan2(-avg.y, avg.x);
	float theta =  -1.0*angle;
	float heading2D = ofRadToDeg(theta)+90;
	
	
	
	l.x = loc.x + noise.x*50;
	l.y = loc.y + noise.y*50;
	l.z = heading2D;
	l += pushLoc;
	
	return l;
	

}
void Boid::draw() {
    // Draw a triangle rotated in the direction of velocity
	 angle = (float)atan2(-vel.y, vel.x);
    float theta =  -1.0*angle;
	float heading2D = ofRadToDeg(theta)+90;
	
	ofPushStyle();
    ofFill();
    ofPushMatrix();
    ofTranslate(loc.x, loc.y);
    ofRotateZ(heading2D);
	ofBeginShape();
    ofVertex(0, -r*2);
    ofVertex(-r, r*2);
    ofVertex(r, r*2);
    ofEndShape(true);	
    ofPopMatrix();
	ofPopStyle();

}

void Boid::flock(vector<Boid> &boids) {
	ofVec2f sep = separate(boids);
//	ofVec2f ali = align(boids);
//	ofVec2f coh = cohesion(boids);
	
	// Arbitrarily weight these forces
	sep *= s;
//	ali *= a;
//	coh *= c;
	
	acc += sep; // + ali + coh; 
}


// Separation
// Method checks for nearby boids and steers away
ofVec2f Boid::separate(vector<Boid> &boids) {
   // float desiredseparation = 75.0f;
    ofVec2f steer;
    int count = 0;
	
    // For every boid in the system, check if it's too close
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		
		float d = ofDist(loc.x, loc.y, other.loc.x, other.loc.y);
		
		// If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
		if ((d > 0) && (d < desiredseparation)) {
			// Calculate vector pointing away from neighbor
			ofVec2f diff = loc - other.loc;
			diff /= d;			// normalize
			diff /= d;        // Weight by distance
			steer += diff;
			count++;            // Keep track of how many
		}
    }
    // Average -- divide by how many
    if (count > 0) {
		steer /= (float)count;
    }
	
	
    // As long as the vector is greater than 0
	//float mag = sqrt(steer.x*steer.x + steer.y*steer.y);
	
	float mag = sqrt(steer.x*steer.x + steer.y*steer.y);
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= maxspeed;
		steer -= vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce);
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
    }
    return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
ofVec2f Boid::align(vector<Boid> &boids) {
    //float neighbordist = 50.0;
    ofVec2f steer;
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		
		float d = ofDist(loc.x, loc.y, other.loc.x, other.loc.y);
		if ((d > 0) && (d < neighbordist)) {
			steer += (other.vel);
			count++;
		}
    }
    if (count > 0) {
		steer /= (float)count;
    }
	
    // As long as the vector is greater than 0
	float mag = sqrt(steer.x*steer.x + steer.y*steer.y);
    if (mag > 0) {
		// Implement Reynolds: Steering = Desired - Velocity
		steer /= mag;
		steer *= maxspeed;
		steer -= vel;
		steer.x = ofClamp(steer.x, -maxforce, maxforce);
		steer.y = ofClamp(steer.y, -maxforce, maxforce);
    }
    return steer;
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
ofVec2f Boid::cohesion(vector<Boid> &boids) {
   // float neighbordist = 50.0;
    ofVec2f sum;   // Start with empty vector to accumulate all locations
    int count = 0;
    for (int i = 0 ; i < boids.size(); i++) {
		Boid &other = boids[i];
		float d = ofDist(loc.x, loc.y, other.loc.x, other.loc.y);
		if ((d > 0) && (d < neighbordist)) {
			sum += other.loc; // Add location
			count++;
		}
    }
    if (count > 0) {
		sum /= (float)count;
		return steer(sum, false);  // Steer towards the location
    
    return sum;
}
	
}
