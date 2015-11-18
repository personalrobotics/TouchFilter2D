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
#include <fstream>

class ofApp;

class AnalyticFilter
{
    public:

        enum FilterMode
        {
            Mode_MPFAnalytic = 0,
            Mode_MPFParticleProjection = 1,
            Mode_MPFBallProjection = 2,
            Mode_MPFUniformProjection = 3,
            Mode_CPF = 4
        };

        const char* ToString(FilterMode mode)
        {
            switch(mode)
            {
                case Mode_MPFAnalytic:
                    return "mpf_analytic";
                    break;
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
        bool ProjectToManifold(const Config& q, Contact& c, Config& out);


        void LoadTrajectory(const std::string& fileName);

        ofVec2f FK(int link, ofVec2f linkPoint, const Config& config);
        std::vector<Config> IK(int link, ofVec2f linkPoint, const ofVec2f& target);
        ofApp* app;
        float* linkLengths;
        Robot trueRobot;
        Config SampleSphere();

        FilterMode filterMode;
        ExperimentMode experimentMode;
        int timestep;
        bool recordData;
        bool recordTrajectory;
        std::fstream dataFile;
        std::fstream trajFile;

        std::vector<Particle> particles;
        std::vector<ofVec2f> points;
        std::vector<Contact> contacts;
        std::vector<Config> recordedTrajectory;
};

#endif /* ANALYTICFILTER_H_ */
