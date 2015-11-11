/*
 * TouchFilter.cpp
 *
 *  Created on: Sep 1, 2015
 *      Author: mklingen
 */

#include "TouchFilter.h"
#include "Definitions.h"
#include "ofApp.h"

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
            Link* link = particle->links[contact.link];
            Link* nextLink = particle->links[contact.link + 1];
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
            TouchFilter::Robot::LinearJacobian J = particle->ComputeLinearJacobian(bodyPoint, contact.link + 1);
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
        world.data.loadImage("demo.png");
        world.distdata.loadImage("demo_dist.png");
        gradientImg.loadImage("jet.png");
        world.Initialize();
        float linkLengths[] = {100.0f, 80.0f, 50.0f};
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

        for (size_t i = 0; i < 100; i++)
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
            ofVec3f sample = SampleBall() * 0.8f;
            offset[0] += sample.x;
            offset[1] += sample.y;
            offset[2] += sample.z;
        }
        cam.setDistance(3);
        mouseDown = false;
    }

    TouchFilter::~TouchFilter()
    {

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

        if((app->mouseX > 0 && app->mouseY > 0) && !mouseDown)
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

            if (CollisionCheck(groundTruthRobot, world))
            {
                ResolveCollision(groundTruthRobot, world);
                //ResampleManifold();
                ResolveContacts();
            }
            else
            {
                WeightParticles();
                ResampleParticles();
            }
        }

        noisyRobot.SetQ(groundTruthRobot.GetQ() + GetJointNoise(groundTruthRobot.GetQ()));

        for (size_t i = 0; i < 100; i++)
        {
            particles.at(i).robot->SetQ(noisyRobot.GetQ() + particles.at(i).offset);
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
        perturbation[0] = 0.1f;
        perturbation[1] = -0.05f;
        perturbation[2] = 0.2f;
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


        ofSetColor(255, 0, 0);
        ofSetLineWidth(1.0);
        for (size_t i = 0; i < gradMag.size(); i++)
        {
            ofLine(gradPts[i], gradPts[i] + gradMag[i] * 10);
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
        for (size_t i = 0; i < particles.size(); i++)
        {
            const Config& qi = particles.at(i).offset;
            Config diff = qi + q * -1.0f;
            BasicMat<1, 1> diffLen = diff.Transpose() * diff;

            density += exp(-1 * sqrt(diffLen[0]));
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

    // Resamples in a box around the current
    // distribution and then projects all the samples
    // to the manifold
    void TouchFilter::ResampleManifold()
    {
        Config minQ;
        minQ[0] = 9999;
        minQ[1] = 9999;
        minQ[2] = 9999;

        Config maxQ;
        maxQ[0] = -9999;
        maxQ[1] = -9999;
        maxQ[2] = -9999;

        for (size_t p = 0; p < particles.size(); p++)
        {
            const Config& qi = particles.at(p).offset;

            minQ[0] = fmin(qi[0], minQ[0]);
            minQ[1] = fmin(qi[1], minQ[1]);
            minQ[2] = fmin(qi[2], minQ[2]);

            maxQ[0] = fmax(qi[0], maxQ[0]);
            maxQ[1] = fmax(qi[1], maxQ[1]);
            maxQ[2] = fmax(qi[2], maxQ[2]);

        }

        float ballSize = 0.05;
        std::vector<Config> newParticles;
        for (size_t p = 0; p < particles.size(); p++)
        {
            Config sample;
            sample[0] = ofRandom(minQ[0] - ballSize, maxQ[0] + ballSize);
            sample[1] = ofRandom(minQ[1] - ballSize, maxQ[1] + ballSize);
            sample[1] = ofRandom(minQ[2] - ballSize, maxQ[2] + ballSize);

            newParticles.push_back(ProjectNLOPT(sample));
        }

        float sumWeights = 0;
        for (size_t p = 0; p < particles.size(); p++)
        {
            Particle& particle = particles.at(p);
            particle.weight = GetKernelDensity(newParticles[p]);
            sumWeights += particle.weight;
        }

        for (size_t p = 0; p < particles.size(); p++)
        {
            Particle& particle = particles.at(p);
            particle.offset = newParticles[p];
            particle.weight /= sumWeights;
        }

        ResampleParticles();

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

    // Projects to the manifold using the jacobian transpose.
    TouchFilter::Config TouchFilter::Project(const Config& q)
    {
        Robot* particle = particles[0].robot;
        Config origOffset = particles[0].offset;

        particle->SetQ(noisyRobot.GetQ() + q);
        particle->Update();
        TouchFilter::Config toReturn = q;

        for (int iter = 0; iter < 10; iter++)
        {
            Robot::Config grad;
            for (size_t i = 0; i < contacts.size(); i++)
            {
                Contact& contact = contacts.at(i);
                Link* link = particle->links[contact.link];
                Link* nextLink = particle->links[contact.link + 1];
                ofVec2f p = link->globalTranslation;
                ofVec2f dir = nextLink->globalTranslation - link->globalTranslation;
                ofVec2f bodyPoint = contact.offset * dir + p;

                ofVec2f g = world.GetGradient((int)bodyPoint.x, (int)bodyPoint.y);
                float d = world.GetDist((int)bodyPoint.x, (int)bodyPoint.y);

                if (d > 0)
                {
                    d*= -1;
                }
                g *= d;
                Robot::LinearJacobian J = particle->ComputeLinearJacobian(bodyPoint, contact.link + 1);
                grad += J.Transpose() * particle->MatFromVec2(g);
            }

            toReturn += grad * -5e-10;
            particle->SetQ(noisyRobot.GetQ() + toReturn);
            particle->Update();
        }

        particle->SetQ(noisyRobot.GetQ() + origOffset);
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
        float ballSize = 0.0005;
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
            ofVec3f sample = SampleBall() * ballSize;
            particles.at(i).offset[0] += sample.x;
            particles.at(i).offset[1] += sample.y;
            particles.at(i).offset[2] += sample.z;
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
                       g *= fabs(world.GetDist((int)bodyPoint.x, (int)bodyPoint.y));

                       Robot::LinearJacobian J = robot.ComputeLinearJacobian(bodyPoint, i + 1);
                       grad += J.Transpose() * robot.MatFromVec2(g);

                       if (k == 9)
                       {
                           gradPts.push_back(bodyPoint);
                           gradMag.push_back(g);
                           Contact contact;
                           contact.link = i;
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
