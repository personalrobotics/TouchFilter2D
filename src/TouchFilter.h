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
                int joint;
                float offset;
                ofVec2f normal;
            };


            enum FilterMode
            {
                Mode_MPFParticleProjection = 1,
                Mode_MPFBallProjection = 2,
                Mode_MPFUniformProjection = 3,
                Mode_CPF = 4
            };

            const char* ToString(FilterMode mode)
            {
                switch(mode)
                {
                    case Mode_MPFBallProjection:
                        return "mpf_ball";
                        break;
                    case Mode_MPFUniformProjection:
                        return "mpf_uniform";
                        break;
                    case Mode_CPF:
                        return "cpf";
                        break;
                    case Mode_MPFParticleProjection:
                        return "mpf_particle";
                        break;
                    default:
                        return "?";
                        break;
                }
                return "?";
            }

            enum ExperimentMode
            {
                Experiment_UserControl,
                Experiment_Dataset
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
            void WeightParticlesContact();
            float GetKernelDensity(const Config& q);

            void ResampleManifold();
            Config Project(const Config& q);
            bool Project(const Config& q, Config& out);
            Config ProjectNLOPT(const Config& q);

            void LoadTrajectory(const std::string& fileName);

            void DrawPlot();

            Config SampleBallUnion(float radius);

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
            std::vector<Config> recordedTrajectory;
            bool recordData;
            bool recordTrajectory;
            std::fstream dataFile;
            std::fstream trajFile;
            FilterMode filterMode;
            ExperimentMode experimentMode;
            size_t timestep;
            float kernelDensity;
            float ballSize;
            float motionDrift;

    };

} /* namespace arm_slam */

#endif /* TOUCHFILTER_H_ */
