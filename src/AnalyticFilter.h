/*
 * AnalyticFilter.h
 *
 *  Created on: Sep 23, 2015
 *      Author: mklingen
 */

#ifndef ANALYTICFILTER_H_
#define ANALYTICFILTER_H_

#include "ofMain.h"
#include "BasicMat.h"
#include "Robot.h"
#include "Common.h"

class ofApp;

class AnalyticFilter
{
    public:
        typedef arm_slam::Robot<2> Robot;
        typedef Robot::Config Config;

        class Particle
        {
            public:
                Robot* robot;
                float weight;

                ~Particle()
                {
                    //delete robot;
                }
        };

        struct Contact
        {
                ofVec2f contactPoint;
                int link;
                ofVec2f linkPoint;
                std::vector<Config> manifold;
        };

        AnalyticFilter();
        virtual ~AnalyticFilter();

        float GetKernelDensity(const Config& q);
        void ComputeManifold(Contact& c);
        void ResolveContacts();
        void Update();
        void Draw();
        void ResampleParticles();
        void MoveRobot(const Config& movement);
        float GetDist(const Config& config, const Contact& contact);
        ofVec2f FK(int link, ofVec2f linkPoint, const Config& config);
        std::vector<Config> IK(int link, ofVec2f linkPoint, const ofVec2f& target);
        ofApp* app;
        float* linkLengths;
        Robot trueRobot;
        Config SampleSphere();

        std::vector<Particle> particles;
        std::vector<ofVec2f> points;
        std::vector<Contact> contacts;
};

#endif /* ANALYTICFILTER_H_ */
