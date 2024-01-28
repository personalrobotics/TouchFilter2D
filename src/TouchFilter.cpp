/*
 * TouchFilter.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: mklingen
 */

#include "TouchFilter.h"
#include "Definitions.h"
#include "ofApp.h"
#include <nlopt.hpp>

namespace arm_slam
{

    // Objective function for nlopt.
    double objective(const std::vector<double>& x, std::vector<double>& gradOut, void* f_data)
    {

        TouchFilter::Robot::Config q;
        q[0] = x[0];
        q[1] = x[1];
        q[2] = x[2];

        TouchFilter* filter = static_cast<TouchFilter*>(f_data);

        TouchFilter::Robot* particle = filter->particles[0].robot;
        TouchFilter::Config origOffset = filter->particles[0].offset;

        particle->SetQ(q);
        particle->Update();
        TouchFilter::Config toReturn = q;

        float cost = 0;
        TouchFilter::Robot::Config grad;
        for (size_t i = 0; i < filter->contacts.size(); i++)
        {
            TouchFilter::Contact& contact = filter->contacts.at(i);
            Link* link = particle->links[contact.joint];
            Link* nextLink = particle->links[contact.joint + 1];
            ofVec2f p = link->globalTranslation;
            ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;
            ofVec2f bodyPoint = contact.offset * dir + p;

            ofVec2f g = filter->world.GetGradient((int)bodyPoint.x, (int)bodyPoint.y);
            float d = filter->world.GetDist((int)bodyPoint.x, (int)bodyPoint.y);

            if (d > 0)
            {
                d*= -1;
            }
            g *= d;
            cost += pow(d, 2);
            TouchFilter::Robot::LinearJacobian J = particle->ComputeLinearJacobian(bodyPoint, contact.joint + 1);
            grad += J.Transpose() * particle->MatFromVec2(g);
        }


        particle->SetQ(filter->noisyRobot.GetQ() + origOffset);
        particle->Update();
        if (!gradOut.empty())
        {
            gradOut[0] = grad[0];
            gradOut[1] = grad[1];
            gradOut[2] = grad[2];
        }

        return (double)cost;
    }

    TouchFilter::TouchFilter() :
            app(0x0)
    {
        ofSeedRandom(0);
        kernelDensity = 8;
        timestep = 0;
        filterMode = TouchFilter::Mode_MPFBallProjection;
        experimentMode = TouchFilter::Experiment_Dataset;
        recordData = true;
        recordTrajectory = false;
        ballSize = 0.05;
        motionDrift = 0.01;
        size_t numParticles = 250;
        float initialError = 0.8f;

        world.data.loadImage("demo.png");
        world.distdata.loadImage("demo_dist.png");
        gradientImg.loadImage("jet.png");
        world.Initialize();
        float linkLengths[] = {100.0f, 80.0f, 50.0f, 1.0f};
        groundTruthRobot.color = ofColor(200, 10, 10);
        groundTruthRobot.Initialize(linkLengths);
        Config c;
        c[0] = 0.5;
        c[1] = 0.0;
        c[2] = 0.0;
        groundTruthRobot.SetQ(c);
        groundTruthRobot.root->localTranslation = ofVec2f(SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
        noisyRobot.Initialize(linkLengths);
        noisyRobot.CloneFrom(groundTruthRobot);
        noisyRobot.SetColor(ofColor(10, 200, 10));

        for (size_t i = 0; i < numParticles; i++)
        {
            Particle particle;
            particle.robot = new Robot();
            particle.robot->Initialize(linkLengths);
            particle.robot->CloneFrom(noisyRobot);
            particle.robot->SetColor(ofColor(100, 100, 255,10));
            particle.weight = 1.0f;
            particle.offset = Config();
            particles.push_back(particle);
        }

        for (size_t i = 0; i < particles.size(); i++)
        {
            Config& offset = particles.at(i).offset;
            ofVec3f sample = SampleBall() * initialError;
            offset[0] += sample.x;
            offset[1] += sample.y;
            offset[2] += sample.z;
        }
        cam.setDistance(3);
        mouseDown = false;

        if (recordTrajectory)
        {
            trajFile.open("traj_tsdf.txt", ios::out | ios::trunc);
        }

        if (recordData)
        {
            std::stringstream ss;
            ss << "error_tsdf_" << ToString(filterMode) << ".txt";
            dataFile.open(ss.str().c_str(), ios::out | ios::trunc);
        }

        if (experimentMode == Experiment_Dataset)
        {
            LoadTrajectory("traj_tsdf.txt");
        }
    }


    void TouchFilter::LoadTrajectory(const std::string& fileName)
    {
        std::ifstream trajFileIn;
        trajFileIn.open(fileName.c_str());
        recordedTrajectory.clear();
        while (trajFileIn.good())
        {
            float q0, q1, q2;
            trajFileIn >> q0;
            trajFileIn >> q1;
            trajFileIn >> q2;
            Config q;
            q[0] = q0;
            q[1] = q1;
            q[2] = q2;
            recordedTrajectory.push_back(q);
        }
        printf("Traj has %lu configs\n", recordedTrajectory.size());
        trajFileIn.close();
    }

    TouchFilter::~TouchFilter()
    {
        trajFile.close();
        dataFile.close();
    }

    void TouchFilter::Update()
    {
        groundTruthRobot.Update();
        noisyRobot.Update();

        for (size_t i = 0; i < 100; i++)
        {
            particles.at(i).robot->Update();
        }

        Config curr = groundTruthRobot.GetQ();

        if(experimentMode == Experiment_UserControl && ((app->mouseX > 0 && app->mouseY > 0) && !mouseDown))
        {
            ofVec2f ee = groundTruthRobot.GetEEPos() * 2;
            ofVec2f force = ee - ofVec2f(app->mouseX, app->mouseY);

            if (force.length() > 10)
            {
                force.normalize();
                force *= 10;
            }

            Robot::Config vel = groundTruthRobot.ComputeJacobianTransposeMove(force);
            groundTruthRobot.SetQ(curr + vel * 1e-5);
            groundTruthRobot.Update();
            noisyRobot.SetQ(groundTruthRobot.GetQ() + GetJointNoise(groundTruthRobot.GetQ()));
            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(noisyRobot.GetQ() + particles.at(i).offset);
                particles.at(i).robot->Update();
            }

        }
        else if (experimentMode == Experiment_Dataset)
        {
            groundTruthRobot.SetQ(recordedTrajectory.at(timestep));
            groundTruthRobot.Update();
            noisyRobot.SetQ(groundTruthRobot.GetQ() + GetJointNoise(groundTruthRobot.GetQ()));
            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(noisyRobot.GetQ() + particles.at(i).offset);
                particles.at(i).robot->Update();
            }

            timestep++;

            if (timestep >= recordedTrajectory.size())
            {
                exit(0);
            }
        }


        if (CollisionCheck(groundTruthRobot, world))
        {
            ResolveCollision(groundTruthRobot, world);
            ResampleManifold();
            //ResolveContacts();
        }
        else
        {
            WeightParticles();
            ResampleParticles();
        }

        for (size_t i = 0; i < particles.size(); i++)
        {
            ofVec3f ballSample = SampleBall() * motionDrift;
            Config noise;
            noise[0] = ballSample.x;
            noise[1] = ballSample.y;
            noise[2] = ballSample.z;
            particles.at(i).offset += noise;
            particles.at(i).robot->SetQ(noisyRobot.GetQ() + particles.at(i).offset);
        }

        if (recordTrajectory)
        {
            Config q = groundTruthRobot.GetQ();
            trajFile << q[0] << " " << q[1] << " " << q[2] << " ";
        }

        if (recordData)
        {
            Config q = groundTruthRobot.GetQ();
            for (size_t i = 0; i < particles.size(); i++)
            {
                Config diff = q - (particles.at(i).offset + noisyRobot.GetQ());
                Mat1x1 difflen = diff.Transpose() * diff;
                float dist = sqrt(difflen[0]);
                dataFile << dist;

                if (i < particles.size() - 1)
                {
                    dataFile << " ";
                }
            }
            dataFile << std::endl;
        }

    }

    void print_result(nlopt::result result)
    {
        switch (result)
        {
            case nlopt::SUCCESS:
                break;
            case nlopt::FTOL_REACHED:
                printf("ftol\n");
                break;
            case nlopt::STOPVAL_REACHED:
                printf("stopval\n");
                break;
            case nlopt::MAXEVAL_REACHED:
                printf("maxeval\n");
                break;
            case nlopt::MAXTIME_REACHED:
                printf("maxtime\n");
                break;
            case nlopt::FORCED_STOP:
                printf("forced\n");
                break;
            default:
                break;
        }
    }

    // Projects a sample to the manifold using NLOPT.
    TouchFilter::Robot::Config TouchFilter::ProjectNLOPT(const Config& q)
    {
        nlopt::opt opt(nlopt::LD_TNEWTON, 3);
        std::vector<double> lower(3);
        lower[0] = q[0] - M_PI;
        lower[1] = q[1] - M_PI;
        lower[2] = q[2] - M_PI;
        std::vector<double> upper(3);
        upper[0] = q[0] + M_PI;
        upper[1] = q[1] + M_PI;
        upper[2] = q[2] + M_PI;
        opt.set_lower_bounds(lower);
        opt.set_upper_bounds(upper);

        opt.set_stopval(50.0);
        opt.set_min_objective(objective, this);
        std::vector<double> q0;
        q0.push_back(q[0]);
        q0.push_back(q[1]);
        q0.push_back(q[2]);
        TouchFilter::Robot::Config c = q;
        try
        {
            double value;
            nlopt::result result = opt.optimize(q0, value);
            c[0] = q0[0];
            c[1] = q0[1];
            c[2] = q0[2];
            return c;
        }
        catch (nlopt::roundoff_limited& e)
        {
            c[0] = q0[0];
            c[1] = q0[1];
            c[2] = q0[2];
            return c;
        }
        catch (std::runtime_error& e)
        {
            // For some reason, runtime error happens
            // in nlopt when no progress can be made
            // in the optimization
            return q;
        }
        catch(std::invalid_argument& e)
        {
            return q;
        }
    }

    // Gets Perlin noise for the given configuration.
    TouchFilter::Robot::Config TouchFilter::GetJointNoise(const TouchFilter::Robot::Config& curr)
    {
        Robot::Config perturbation;
        //perturbation[0] = 0.25f * (ofNoise((float)curr(0), (float)curr(1), (float)curr(2) + 0.5f) - 0.5f);
        //perturbation[1] = 0.25f * (ofNoise((float)curr(0), (float)curr(1) + 0.5f, (float)curr(2)) - 0.5f);
        //perturbation[2] = 0.25f * (ofNoise((float)curr(0) + 0.5f, (float)curr(1), (float)curr(2)) - 0.5f);
        perturbation[0] = 0.0f;
        perturbation[1] = -0.0f;
        perturbation[2] = 0.0f;
        return perturbation;
    }

    // Draws the robot/world
    void TouchFilter::Draw()
    {
        ofScale(2.0, 2.0, 1.0);
        ofSetColor(255);
        world.data.draw(0, 0);
        noisyRobot.Draw();
        groundTruthRobot.Draw();

        for (size_t i = 0; i < 100; i++)
        {
            particles.at(i).robot->Draw();
        }

        /*
        ofSetLineWidth(1.0);
        ofSetColor(255, 0, 0);
        for (int x = 1; x < world.distdata.width - 1; x+=10)
        {
            for (int y = 1; y < world.distdata.height - 1; y+=10)
            {
                ofVec2f grad = world.GetGradient(x, y);
                grad.normalize();
                ofLine(ofVec2f(x, y), ofVec2f(x, y) + grad * 5);
            }
        }
        */


        ofSetColor(255, 0, 0);
        ofSetLineWidth(1.0);
        for (size_t i = 0; i < gradMag.size(); i++)
        {
            ofVec3f start = ofVec3f(gradPts[i].x, gradPts[i].y, 0.0);
            ofVec3f end = ofVec3f(gradPts[i].x + gradMag[i].x * 10, gradPts[i].y + gradMag[i].y * 10, 0.0);
            ofLine(start, end);
            // ofLine(gradPts[i], gradPts[i] + gradMag[i] * 10);
        }
        gradMag.clear();
        gradPts.clear();
        DrawPlot();
    }

    // Computes a kernel density estimate for a given configuration
    // from the particle distribution
    float TouchFilter::GetKernelDensity(const Config& q)
    {
        float density = 0;
        ofVec3f qvec(q[0], q[1], q[2]);
        for (size_t i = 0; i < particles.size(); i++)
        {
            const Config& qi = particles.at(i).offset;
            ofVec3f qivec(qi[0], qi[1], qi[2]);
            density += exp(-kernelDensity * TorusDist(qvec, qivec));
        }
        return density;
    }

    // This weights the particles in the case when there is NO contact.
    void TouchFilter::WeightParticles()
    {
        float sumWeights = 0;
        for (size_t p = 0; p < particles.size(); p++)
        {
            float cost = 0.0f;
            Particle& particle = particles.at(p);
            for (size_t i = 0; i < 3; i++)
            {
                Link* link = particle.robot->links[i];
                Link* nextLink = particle.robot->links[i + 1];

                ofVec2f p = link->globalTranslation;
                ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;

                for (float t = 0; t < 1.0; t += 0.1f)
                {
                    ofVec2f bodyPoint = t * dir + p;
                    if (!world.IsValid((int)bodyPoint.x, (int)bodyPoint.y))
                    {
                        continue;
                    }
                    else if (world.Collides((int)bodyPoint.x, (int)bodyPoint.y))
                    {
                        cost += -0.1 * world.GetDist((int)bodyPoint.x, (int)bodyPoint.y);
                    }
                }
            }

            particle.weight = exp(-cost);
            sumWeights += particle.weight;
        }

        for (size_t p = 0; p < particles.size(); p++)
        {
            particles.at(p).weight /= sumWeights;
        }
    }

    void TouchFilter:: WeightParticlesContact()
    {
        float sumWeights = 0;
        float cpf_weight = 0.1;
        for (size_t p = 0; p < particles.size(); p++)
        {
            float cost = 0.0f;
            Particle& particle = particles.at(p);
            //for (size_t i = 0; i < 3; i++)
            for (size_t c = 0; c < contacts.size(); c++)
            {
                const Contact& contact = contacts.at(c);
                Link* link = particle.robot->links[contact.joint];
                Link* nextLink = particle.robot->links[contact.joint + 1];

                ofVec2f p = link->globalTranslation;
                ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;

                ofVec2f bodyPoint = contact.offset * dir + p;
                cost += pow(world.GetDist((int)bodyPoint.x, (int)bodyPoint.y), 2);
            }

            particle.weight = exp(-cpf_weight * cost);
            sumWeights += particle.weight;
        }

        for (size_t p = 0; p < particles.size(); p++)
        {
            particles.at(p).weight /= sumWeights;
        }
    }

    // Resamples from the contact manifold
    void TouchFilter::ResampleManifold()
    {
        Config curr = noisyRobot.GetQ();
        switch (filterMode)
        {
            case (Mode_MPFUniformProjection) :
            {
                std::vector<Config> newParticles;
                for (size_t p = 0; p < particles.size(); p++)
                {
                    Config sample;
                    Config out;
                    bool success = false;
                    do
                    {
                        sample[0] = ofRandom(-M_PI * 0.5, M_PI * 0.5);
                        sample[1] = ofRandom(-M_PI * 0.5, M_PI * 0.5);
                        sample[2] = ofRandom(-M_PI * 0.5, M_PI * 0.5);
                        success = Project(sample + curr, out);
                    } while (!success);

                    newParticles.push_back(out);
                }

                float sumWeights = 0;
                for (size_t p = 0; p < particles.size(); p++)
                {
                    Particle& particle = particles.at(p);
                    particle.weight = GetKernelDensity(newParticles[p] - curr);
                    sumWeights += particle.weight;
                }

                for (size_t p = 0; p < particles.size(); p++)
                {
                    Particle& particle = particles.at(p);
                    particle.offset = newParticles[p] - curr;
                    particle.weight /= sumWeights;
                }

                ResampleParticles();
                break;
            }
            case (Mode_MPFParticleProjection):
            {
                std::vector<Config> newParticles;
                for (size_t p = 0; p < particles.size(); p++)
                {
                    newParticles.push_back(Project(particles.at(p).offset + curr));
                }

                float sumWeights = 0;
                for (size_t p = 0; p < particles.size(); p++)
                {
                    Particle& particle = particles.at(p);
                    particle.weight = GetKernelDensity(newParticles[p] - curr);
                    sumWeights += particle.weight;
                }

                for (size_t p = 0; p < particles.size(); p++)
                {
                    Particle& particle = particles.at(p);
                    particle.offset = newParticles[p] - curr;
                    particle.weight /= sumWeights;
                }

                ResampleParticles();
                break;
            }
            case (Mode_MPFBallProjection):
            {
                std::vector<Config> newParticles;
                for (size_t p = 0; p < particles.size(); p++)
                {
                    Config s = SampleBallUnion(ballSize);
                    newParticles.push_back(Project(s + curr));
                }

                float sumWeights = 0;
                for (size_t p = 0; p < particles.size(); p++)
                {
                    Particle& particle = particles.at(p);
                    particle.weight = GetKernelDensity(newParticles[p] - curr);
                    sumWeights += particle.weight;
                }

                for (size_t p = 0; p < particles.size(); p++)
                {
                    Particle& particle = particles.at(p);
                    particle.offset = newParticles[p] - curr;
                    particle.weight /= sumWeights;
                }

                ResampleParticles();
                break;
            }
            case (Mode_CPF):
            {
                WeightParticlesContact();
                ResampleParticles();
                break;
            }
        }
        contacts.clear();

    }

    TouchFilter::Config TouchFilter::SampleBallUnion(float radius)
    {
        TouchFilter::Config minConfig;
        minConfig[0] = HUGE_VAL;
        minConfig[1] = HUGE_VAL;
        minConfig[2] = HUGE_VAL;
        TouchFilter::Config maxConfig;
        maxConfig[0] = -HUGE_VAL;
        maxConfig[1] = -HUGE_VAL;
        maxConfig[2] = -HUGE_VAL;
        for (size_t i = 0; i < particles.size(); i++)
        {
            const Particle& p = particles.at(i);
            for (int k = 0; k < 3; k++)
            {
                minConfig[k] = fmin(minConfig[k], p.offset[k]);
                maxConfig[k] = fmax(maxConfig[k], p.offset[k]);
            }
        }

        TouchFilter::Config sample;
        float dist = HUGE_VAL;
        do
        {
            for (int k = 0; k < 3; k++)
                sample[k] = ofRandom(minConfig[k] - radius, maxConfig[k] + radius);

            for (size_t i = 0; i < particles.size(); i++)
            {
                const Particle& p = particles.at(i);
                dist = TorusDist(ofVec3f(sample[0], sample[1], sample[2]), ofVec3f(p.offset[0], p.offset[1], p.offset[2]));

                if (dist < radius) return sample;
            }

        }
        while(dist > radius);
        return sample;
    }

    // Returns whether the robot collides with the world.
    bool TouchFilter::CollisionCheck( Robot& robot,  World& world)
    {
        for (size_t i = 0; i < 3; i++)
        {
            Link* link = robot.links[i];
            Link* nextLink = robot.links[i + 1];

            ofVec2f p = link->globalTranslation;
            ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;

            for (float t = 0; t < 1.0; t += 0.1f)
            {
                ofVec2f bodyPoint = t * dir + p;
                if (!world.IsValid((int)bodyPoint.x, (int)bodyPoint.y))
                {
                    return true;
                }
                else if (world.Collides((int)bodyPoint.x, (int)bodyPoint.y))
                {
                    return true;
                }
            }
        }

        return false;
    }

    bool TouchFilter::Project(const Config& q, Config& out)
    {
        Robot* particle = particles[0].robot;
        Config origOffset = particles[0].offset;

        particle->SetQ(q);
        particle->Update();
        TouchFilter::Config toReturn = q;


        float mult = -1e-5 / contacts.size();
        bool anyError = false;
        for (int iter = 0; iter < 100; iter++)
        {
          anyError = false;
          Robot::Config grad;
          for (size_t i = 0; i < contacts.size(); i++)
          {
              Contact& contact = contacts.at(i);

              Link* link = particle->links[contact.joint];
              Link* nextLink = particle->links[contact.joint + 1];
              ofVec2f p = link->globalTranslation;
              ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;
              ofVec2f bodyPoint = contact.offset * dir + p;

              ofVec2f g = world.GetGradient((int)bodyPoint.x, (int)bodyPoint.y);
              float d = world.GetDist((int)bodyPoint.x, (int)bodyPoint.y);

              if (fabs(d) > 1.5)
              {
                  Robot::LinearJacobian J = particle->ComputeLinearJacobian(bodyPoint, contact.joint);
                  grad += d * J.Transpose() * particle->MatFromVec2(g);
                  anyError = true;
              }
              else
              {
                  break;
              }
          }
          toReturn -= (grad * mult);
          particle->SetQ(toReturn);
          particle->Update();
          if (!anyError)
          {
              break;
          }
        }

        particle->SetQ(noisyRobot.GetQ() + origOffset);
        out = toReturn;
        return !anyError;
    }

    // Projects to the manifold using the jacobian transpose.
    TouchFilter::Config TouchFilter::Project(const Config& q)
    {
        TouchFilter::Config toReturn = q;
        Project(q, toReturn);
        return toReturn;
    }

    // This one just projects the current distribution
    // to the contact manifold.
    void TouchFilter::ResolveContacts()
    {
        Robot::Config curr = noisyRobot.GetQ();
        for (size_t p = 0; p < particles.size(); p++)
        {
            Config& offset = particles.at(p).offset;
            Robot::Config q;
            q = ProjectNLOPT(curr + offset);
            offset = q - curr;
        }
        contacts.clear();
    }

    // Sequential importance resampling + fuzz
    void TouchFilter::ResampleParticles()
    {
        std::vector<Config> offsets;

        for (size_t i = 0; i < particles.size(); i++)
        {
            offsets.push_back(particles.at(i).offset);
        }

        float M = particles.size();
        float Minv = 1.0f / M;
        float r = ofRandom(Minv);
        float t = r;
        for (size_t i = 0; i < particles.size(); i++)
        {
            int j = 0;
            float currT = 0;
            for (size_t k = 0; k < particles.size(); k++)
            {
                currT += particles.at(k).weight;
                if (currT > t) { j = (int)k; break; }
            }
            particles.at(i).offset = offsets.at(j);
            t += Minv;
        }
    }

    // Draws the 3D plot in the upper left corner.
    void TouchFilter::DrawPlot()
    {
        cam.setNearClip(0.001);
        cam.setFarClip(10.0f);

        cam.begin(ofRectangle(0, 0, 512, 512));
        ofSetColor(255, 0, 0);
        ofDrawArrow(ofVec3f(-3, -3, -3), ofVec3f(3, -3, -3));
        ofSetColor(0, 255, 0);
        ofDrawArrow(ofVec3f(-3, -3, -3), ofVec3f(-3, 3, -3));
        ofSetColor(0, 0, 255);
        ofDrawArrow(ofVec3f(-3, -3, -3), ofVec3f(-3, -3, 3));

        Config q = noisyRobot.GetQ() ;
        Config q0 = groundTruthRobot.GetQ();
        Config noise = q +  q0 * -1.0f;
        ofSetColor(0, 255, 0);
        ofDrawSphere(-noise(0), -noise(1), -noise(2), 0.03);

        ofSetColor(0, 0, 255, 5);

        for (size_t i = 0; i < particles.size(); i++)
        {
            Config pos = q + particles.at(i).offset;
            ofDrawSphere(particles.at(i).offset(0), particles.at(i).offset(1), particles.at(i).offset(2), 0.02);
        }

        cam.end();
    }

    // Pushes the robot out of collision and computes contacts.
    void TouchFilter::ResolveCollision(Robot& robot, World& world)
    {

        Robot::Config grad;
        int num = 0;
        for (int k = 0; k < 10; k++)
        {
            for (size_t i = 0; i < 3; i++)
            {
                Link* link = robot.links[i];
                Link* nextLink = robot.links[i + 1];

                ofVec2f p = link->globalTranslation;
                ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;
                for (float t = 0; t < 1.0; t += 0.1f)
                {
                    ofVec2f bodyPoint = t * dir + p;
                    if (!world.IsValid((int)bodyPoint.x, (int)bodyPoint.y))
                    {

                    }
                    else if (world.Collides((int)bodyPoint.x, (int)bodyPoint.y))
                    {
                       ofVec2f g = world.GetGradient((int)bodyPoint.x, (int)bodyPoint.y);
                       g *= pow(world.GetDist((int)bodyPoint.x, (int)bodyPoint.y), 2);

                       Robot::LinearJacobian J = robot.ComputeLinearJacobian(bodyPoint, i + 1);
                       grad -= J.Transpose() * robot.MatFromVec2(g);

                       if (k == 9)
                       {
                           gradPts.push_back(bodyPoint);
                           gradMag.push_back(g);
                           Contact contact;
                           contact.joint = i;
                           contact.offset = t;
                           contact.normal = g;
                           contacts.push_back(contact);
                       }
                       num++;
                    }
                }
            }
            if (num)
            {
                float normalization = 0.000001f / num;
                robot.SetQ(robot.GetQ() + grad * normalization);
            }
        }

    }

} /* namespace arm_slam */
