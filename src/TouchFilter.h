/*
 * TouchFilter.h
 *
 *  Created on: Sep 1, 2015
 *      Author: mklingen
 */

#ifndef TOUCHFILTER_H_
#define TOUCHFILTER_H_

#include "ofMain.h"
#include "Robot.h"
#include "TSDF.h"
#include "World.h"
#include <nlopt.hpp>

class ofApp;

namespace arm_slam
{

    class TouchFilter
    {
        public:

            struct Contact
            {
                int link;
                float offset;
                ofVec2f normal;
            };


            typedef arm_slam::Robot<3> Robot;
            typedef Robot::Config Config;


            struct Particle
            {
                    Robot* robot;
                    Config offset;
                    float weight;
            };

            TouchFilter();
            virtual ~TouchFilter();

            void Update();
            void Draw();
            Config GetJointNoise(const Config& curr);

            bool CollisionCheck( Robot& robot,  World& world);
            void ResolveCollision(Robot& robot, World& world);
            void ResolveContacts();

            void ResampleParticles();

            void WeightParticles();
            float GetKernelDensity(const Config& q);

            void ResampleManifold();
            Config Project(const Config& q);
            Config ProjectNLOPT(const Config& q);

            void DrawPlot();

            World world;
            Robot groundTruthRobot;
            Robot noisyRobot;
            ofImage gradientImg;
            ofApp* app;
            ofEasyCam cam;
            std::vector<ofVec2f> gradPts;
            std::vector<ofVec2f> gradMag;
            std::vector<Contact> contacts;
            std::vector<Particle> particles;
            bool mouseDown;

    };

} /* namespace arm_slam */

#endif /* TOUCHFILTER_H_ */
