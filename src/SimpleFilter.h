/*
 * TouchFilter.h
 *
 *  Created on: Sep 1, 2015
 *      Author: mklingen
 */

#ifndef SIMPLEFILTER_H_
#define SIMPLEFILTER_H_

#include "ofMain.h"
#include <map>
#include "Common.h"

class ofApp;

class SimpleFilter
{
    public:
        struct Particle
        {
            float weight;
            ofVec3f pos;
        };

        SimpleFilter();
        virtual ~SimpleFilter();

        Particle& GetClosest(const ofVec3f& pos);
        ofVec3f UniformSampleManifold(ofVec3f minBounds, ofVec3f maxBounds);
        void Update();
        void Draw();
        void ResampleParticles();
        void WeightParticles();
        void DrawPlot();
        float GetManifold(float x, float y);
        void MoveParticles(ofVec3f movement);
        void Pressed(int key);
        void Released(int key);

        float GetKernelDensity(const ofVec3f& pos);

        ofApp* app;
        ofEasyCam cam;
        std::vector<Particle> particles;
        std::map<int, bool> keys;
        ofVec3f truePos;
        bool colliding;

};

#endif /* TOUCHFILTER_H_ */
