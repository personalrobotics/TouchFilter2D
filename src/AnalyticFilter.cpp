/*
 * AnalyticFilter.cpp
 *
 *  Created on: Sep 23, 2015
 *      Author: mklingen
 */
#include "ofApp.h"
#include "AnalyticFilter.h"
#include "Definitions.h"

#include <sstream>

AnalyticFilter::AnalyticFilter()
{
    timestep = 0;
    experimentMode = Experiment_Dataset;
    filterMode = Mode_MPFAnalytic;
    linkLengths = new float[3];
    linkLengths[0] = 100;
    linkLengths[1] = 50;
    linkLengths[2] = 1;
    trueRobot.Initialize(linkLengths);
    trueRobot.SetColor(ofColor(0));
    recordData = true;
    recordTrajectory = false;

    float x = SCREEN_WIDTH / 2;
    float y = SCREEN_HEIGHT / 2;
    trueRobot.root->localTranslation.x = x;
    trueRobot.root->localTranslation.y = y;
    Config zero;
    zero[0] = 0;
    zero[1] = 0;
    ofSeedRandom(0);
    trueRobot.SetQ(zero);
    trueRobot.Update();
    points.push_back(ofVec2f(x + 100, y - 50));
    // for (int dx = 0; dx < SCREEN_WIDTH; dx+=10)
    //{
    //     float dy = x + ofNoise(dx * 0.01) * 300 - 60;
    //     points.push_back(ofVec2f(dx, dy));
    // }

    if (experimentMode == Experiment_Dataset)
    {
        LoadTrajectory("./trajectory.txt");
        trueRobot.SetQ(recordedTrajectory[0]);
    }

    int numParticles = 350;
    float fuzz = 2.8f;
    for (int i = 0; i < numParticles; i++)
    {
        Particle p;
        p.robot = new Robot();
        p.robot->Initialize(linkLengths);
        p.robot->CloneFrom(trueRobot);
        p.robot->SetColor(ofColor(100, 100, 255, 10));
        p.robot->SetQ(trueRobot.GetQ() + SampleSphere() * fuzz);
        p.weight = 1.0f;
        particles.push_back(p);
    }

    if (recordData)
    {
        std::stringstream ss;
        ss << "./error_" << ToString(filterMode) << ".txt";
        dataFile.open(ss.str().c_str(), ios_base::out | ios_base::trunc);
    }

    if (recordTrajectory)
    {
        trajFile.open("./trajectory.txt", ios_base::in | ios_base::out | ios_base::trunc);
        if (trajFile.fail())
        {
            cerr << "open failure: " << strerror(errno) << '\n';
            exit(-1);
        }
    }
}

AnalyticFilter::~AnalyticFilter()
{
    dataFile.close();
    trajFile.close();
}

AnalyticFilter::Config AnalyticFilter::SampleSphere()
{
    ofVec2f p = SampleDisc();

    Config c;
    c(0) = p.x;
    c(1) = p.y;
    return c;
}

void AnalyticFilter::Update()
{
    timestep++;
    switch (experimentMode)
    {
    case Experiment_UserControl:
    {
        if ((app->mouseX > 0 && app->mouseY > 0))
        {
            trueRobot.Update();
            ofVec2f ee = trueRobot.GetEEPos() * 2;
            ofVec2f force = ee - ofVec2f(app->mouseX, app->mouseY);

            Robot::Config vel = trueRobot.ComputeJacobianTransposeMove(force);

            if (!(isinf(vel[0]) || isinf(vel[1])) && (fabs(vel[0]) + fabs(vel[1])) < 100000)
            {
                MoveRobot(vel * 1e-5);
            }
        }
        break;
    }

    case Experiment_Dataset:
    {
        size_t idx = timestep;
        if (idx < recordedTrajectory.size())
        {
            Config q = recordedTrajectory[idx];
            Config curr = trueRobot.GetQ();

            trueRobot.Update();

            MoveRobot(q - curr);
        }
        break;
    }
    }
}

void AnalyticFilter::LoadTrajectory(const std::string &fileName)
{
    std::ifstream trajFileIn;
    trajFileIn.open(fileName.c_str());
    recordedTrajectory.clear();
    while (trajFileIn.good())
    {
        float q0, q1;
        trajFileIn >> q0;
        trajFileIn >> q1;
        Config q;
        q[0] = q0;
        q[1] = q1;
        recordedTrajectory.push_back(q);
    }
    printf("Traj has %lu configs\n", recordedTrajectory.size());
    trajFileIn.close();
}

void AnalyticFilter::ResolveContacts()
{
    for (size_t i = 0; i < points.size(); i++)
    {
        for (size_t l = 0; l < 2; l++)
        {
            for (float t = 0; t < 1.0f; t += 0.05f)
            {
                ofVec2f localPoint(t, 0);
                ofVec2f globalPoint = FK(l, localPoint, trueRobot.GetQ());

                if (globalPoint.distance(points.at(i)) < 1.5f)
                {
                    Contact c;
                    c.link = l;
                    c.linkPoint = localPoint;
                    c.contactPoint = points.at(i);
                    ComputeManifold(c);
                    contacts.push_back(c);
                }
            }
        }
    }

    if (contacts.size() > 0)
    {
        std::vector<Config> newParticles;
        std::vector<float> newWeights;
        float sum = 0;
        switch (filterMode)
        {
        case (Mode_CPF):
        {
            printf("CPF!\n");
            for (size_t i = 0; i < particles.size(); i++)
            {
                size_t c = (size_t)ofRandom(contacts.size());
                float dist = GetDist(particles.at(i).robot->GetQ(), contacts.at(c));
                newParticles.push_back(particles.at(i).robot->GetQ());
                newWeights.push_back(exp(-0.1 * dist));
                sum += newWeights.back();
            }

            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(newParticles.at(i));
                particles.at(i).weight = newWeights.at(i) / sum;
            }

            ResampleParticles();

            break;
        }
        case (Mode_MPFAnalytic):
        {
            for (size_t i = 0; i < particles.size(); i++)
            {
                size_t c = (size_t)ofRandom(contacts.size());
                size_t m = (size_t)ofRandom(contacts.at(c).manifold.size());
                if (contacts.at(c).manifold.size() > 0)
                {
                    newParticles.push_back(contacts.at(c).manifold.at(m));
                    newWeights.push_back(GetKernelDensity(contacts.at(c).manifold.at(m)));
                    sum += newWeights.back();
                }
                else
                {
                    newParticles.push_back(particles.at(i).robot->GetQ());
                    newWeights.push_back(GetKernelDensity(particles.at(i).robot->GetQ()));
                    sum += newWeights.back();
                }
            }

            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(newParticles.at(i));
                particles.at(i).weight = newWeights.at(i) / sum;
            }

            ResampleParticles();
            break;
        }
        case (Mode_MPFBallProjection):
        {
            for (size_t i = 0; i < particles.size(); i++)
            {
                size_t c = (size_t)ofRandom(contacts.size());
                if (contacts.at(c).manifold.size() > 0)
                {
                    Config out = particles.at(i).robot->GetQ();
                    out += SampleSphere() * 0.05;
                    ProjectToManifold(out, contacts.at(c), out);
                    newParticles.push_back(out);
                    newWeights.push_back(GetKernelDensity(out));
                    sum += newWeights.back();
                }
                else
                {
                    newParticles.push_back(particles.at(i).robot->GetQ());
                    newWeights.push_back(GetKernelDensity(particles.at(i).robot->GetQ()));
                }
            }

            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(newParticles.at(i));
                particles.at(i).weight = newWeights.at(i) / sum;
            }

            ResampleParticles();
            break;
        }
        case (Mode_MPFParticleProjection):
        {
            for (size_t i = 0; i < particles.size(); i++)
            {
                size_t c = (size_t)ofRandom(contacts.size());
                if (contacts.at(c).manifold.size() > 0)
                {
                    Config out;
                    ProjectToManifold(particles.at(i).robot->GetQ(), contacts.at(c), out);
                    newParticles.push_back(out);
                    newWeights.push_back(GetKernelDensity(out));
                    sum += newWeights.back();
                }
                else
                {
                    newParticles.push_back(particles.at(i).robot->GetQ());
                    newWeights.push_back(GetKernelDensity(particles.at(i).robot->GetQ()));
                }
            }

            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(newParticles.at(i));
                particles.at(i).weight = newWeights.at(i) / sum;
            }

            ResampleParticles();
            break;
        }
        case (Mode_MPFUniformProjection):
        {
            for (size_t i = 0; i < particles.size(); i++)
            {
                size_t c = (size_t)ofRandom(contacts.size());
                if (contacts.at(c).manifold.size() > 0)
                {
                    Config out = SampleSphere() * 6;
                    ProjectToManifold(out, contacts.at(c), out);
                    newParticles.push_back(out);
                    newWeights.push_back(GetKernelDensity(out));
                    sum += newWeights.back();
                }
                else
                {
                    newParticles.push_back(particles.at(i).robot->GetQ());
                    newWeights.push_back(GetKernelDensity(particles.at(i).robot->GetQ()));
                }
            }

            for (size_t i = 0; i < particles.size(); i++)
            {
                particles.at(i).robot->SetQ(newParticles.at(i));
                particles.at(i).weight = newWeights.at(i) / sum;
            }

            ResampleParticles();
            break;
        }
        }
    }
}

bool AnalyticFilter::ProjectToManifold(const Config &config, Contact &c, Config &out)
{
    Config q = config;
    out = config;
    Config orig = trueRobot.GetQ();
    float threshold = 1;
    for (int iter = 0; iter < 100; iter++)
    {
        trueRobot.SetQ(q);
        trueRobot.Update();
        out = q;
        ofVec2f fk = FK(c.link, c.linkPoint, q);
        ofVec2f diff = (c.contactPoint - fk);

        if (diff.length() > threshold)
        {
            Robot::LinearJacobian jacobian = trueRobot.ComputeLinearJacobian(fk, c.link + 1);

            Config grad = 1e-4 * jacobian.Transpose() * trueRobot.MatFromVec2(diff);
            q -= grad;
            out = q;
        }
        else
        {
            trueRobot.SetQ(orig);
            trueRobot.Update();
            return true;
        }
    }
    trueRobot.SetQ(orig);
    trueRobot.Update();
    return false;
}

void AnalyticFilter::Draw()
{
    ofScale(2, 2, 1);
    ofClear(255);

    trueRobot.Draw();

    for (size_t i = 0; i < particles.size(); i++)
    {
        particles.at(i).robot->Draw();
    }

    ofSetColor(0);

    for (size_t i = 0; i < points.size(); i++)
    {
        ofCircle(points.at(i).x, points.at(i).y, 1.0f);
    }
    float pi2 = 2 * 3.14159;
    float scale = 40;
    float a = 0;
    float b = pi2 * scale;
    ofSetLineWidth(1.0f);
    ofSetColor(0);
    ofLine(0, 0, 0, pi2 * scale);
    ofLine(0, 0, pi2 * scale, 0);
    ofLine(pi2 * scale, 0, pi2 * scale, pi2 * scale);
    ofLine(pi2 * scale, pi2 * scale, 0, pi2 * scale);
    Config q = trueRobot.GetQ();
    ofRect(wrap(q(0) * scale, a, b), wrap(q(1) * scale, a, b), 2, 2);

    trueRobot.SetColor(ofColor(0, 0, 0, 10));
    for (size_t i = 0; i < contacts.size(); i++)
    {
        ofSetColor(255, 0, 0);
        ofCircle(contacts.at(i).contactPoint.x, contacts.at(i).contactPoint.y, 3.0f);

        for (size_t m = 0; m < contacts.at(i).manifold.size(); m++)
        {
            ofSetColor(255, 0, 0);
            ofRect(wrap(contacts.at(i).manifold.at(m)(0) * scale, a, b) - 2, wrap(contacts.at(i).manifold.at(m)(1) * scale, a, b) - 2, 4, 4);
        }
    }

    for (size_t i = 0; i < particles.size(); i++)
    {
        ofSetColor(100, 100, 255, 10);
        Config qi = particles.at(i).robot->GetQ();
        ofRect(wrap(qi(0) * scale, a, b), wrap(qi(1) * scale, a, b), 2, 2);
    }

    trueRobot.SetQ(q);
    trueRobot.SetColor(ofColor(0));

    /*
    for (size_t l = 0; l < 2; l++)
    {
       for (float t = 0; t < 1.0f; t += 0.05f)
       {
           ofVec2f localPoint(t, 0);
           ofVec2f globalPoint = FK(l, localPoint, trueRobot.GetQ());

           ofCircle(globalPoint, 3.0f);
       }
    }
    */

    contacts.clear();
}

void AnalyticFilter::ResampleParticles()
{
    float ballSize = 0.01;
    std::vector<Config> offsets;

    for (size_t i = 0; i < particles.size(); i++)
    {
        offsets.push_back(particles.at(i).robot->GetQ());
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
            if (currT > t)
            {
                j = (int)k;
                break;
            }
        }
        particles.at(i).robot->SetQ(offsets.at(j) + SampleSphere() * ballSize);
        t += Minv;
    }
}

float AnalyticFilter::GetKernelDensity(const Config &q)
{
    ofVec2f qp(q(0), q(1));
    float density = 0;
    for (size_t i = 0; i < particles.size(); i++)
    {
        const Config &qi = particles.at(i).robot->GetQ();
        ofVec2f pi(qi(0), qi(1));
        density += exp(-27.57 * TorusDist(pi, qp)) / particles.size();
    }

    return density;
}

void AnalyticFilter::MoveRobot(const Config &movement)
{

    float pi = 3.14159;
    trueRobot.SetQ(trueRobot.GetQ() + movement);
    trueRobot.Update();
    Config q = trueRobot.GetQ();

    if (recordTrajectory)
    {
        trajFile << q[0] << " " << q[1];
        printf("%f %f\n", movement[0], movement[1]);
    }

    for (size_t i = 0; i < particles.size(); i++)
    {
        Config qnew = particles.at(i).robot->GetQ() + movement + SampleSphere() * 0.005;
        qnew(0) = wrap(qnew(0), -pi, pi);
        qnew(1) = wrap(qnew(1), -pi, pi);
        particles.at(i).robot->SetQ(qnew);
        particles.at(i).robot->Update();
    }
    ResolveContacts();

    if (recordData)
    {
        ofVec2f qReal(q[0], q[1]);
        for (size_t i = 0; i < particles.size(); i++)
        {
            Config qi = particles.at(i).robot->GetQ();
            ofVec2f veci(qi[0], qi[1]);
            float error = TorusDist(qReal, veci);
            dataFile << error;

            if (i < particles.size() - 1)
            {
                dataFile << " ";
            }
        }
        dataFile << std::endl;
    }
}

float AnalyticFilter::GetDist(const Config &config, const Contact &contact)
{
    ofVec2f fk = FK(contact.link, contact.linkPoint, config);
    return fk.distance(contact.contactPoint);
}

ofVec2f AnalyticFilter::FK(int link, ofVec2f linkPoint, const Config &config)
{
    Config orig = trueRobot.GetQ();
    trueRobot.SetQ(config);
    trueRobot.Update();
    if (link > 0)
    {
        const arm_slam::Link *lk = trueRobot.links[link - 1];
        const arm_slam::Link *nextLink = trueRobot.links[link];
        ofVec2f dir = nextLink->globalTranslation - lk->globalTranslation;
        ofVec2f globalPoint = lk->globalTranslation + dir * linkPoint.x;
        trueRobot.SetQ(orig);
        trueRobot.Update();
        return globalPoint;
    }
    else
    {
        const arm_slam::Link *nextLink = trueRobot.links[link];
        ofVec2f dir = nextLink->globalTranslation - trueRobot.root->globalTranslation;
        ofVec2f globalPoint = trueRobot.root->globalTranslation + dir * linkPoint.x;
        trueRobot.SetQ(orig);
        trueRobot.Update();
        return globalPoint;
    }
}

std::vector<AnalyticFilter::Config> AnalyticFilter::IK(int link, ofVec2f linkPoint, const ofVec2f &target)
{
    float pi2 = 2 * 3.14159;
    float y = trueRobot.root->globalTranslation.y - target.y;
    float x = target.x - trueRobot.root->globalTranslation.x;
    std::vector<AnalyticFilter::Config> iks;
    if (link == 0)
    {
        float q0 = atan2(y, x);
        for (float q1 = 0; q1 < pi2; q1 += 0.025f)
        {
            Config c;
            c[0] = q0;
            c[1] = q1;
            iks.push_back(c);
        }

        return iks;
    }
    else
    {
        float l1 = linkLengths[0];
        float l2 = linkPoint.x * linkLengths[1];

        float YY = 1.0f - pow((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2), 2);
        float Y = sqrt(fmax(YY, 0.0f));
        float X = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        {
            float q2 = atan2(Y, X);
            float k1 = l1 + l2 * cos(q2);
            float k2 = l2 * sin(q2);
            float gamma = atan2(k2, k1);

            float q1 = atan2(y, x) - gamma;
            Config c;
            c[0] = q1;
            c[1] = q2;
            iks.push_back(c);
        }
        {
            float q2 = atan2(-Y, X);
            float k1 = l1 + l2 * cos(q2);
            float k2 = l2 * sin(q2);
            float gamma = atan2(k2, k1);

            float q1 = atan2(y, x) - gamma;
            Config c;
            c[0] = q1;
            c[1] = q2;
            iks.push_back(c);
        }
        return iks;
    }
}

void AnalyticFilter::ComputeManifold(AnalyticFilter::Contact &c)
{
    Config q = trueRobot.GetQ();
    std::vector<Config> iks = IK(c.link, c.linkPoint, c.contactPoint);

    for (size_t i = 0; i < iks.size(); i++)
    {
        trueRobot.SetQ(iks.at(i));
        trueRobot.Update();
        // trueRobot.Draw();
        c.manifold.push_back(iks.at(i));
    }
    /*
    for (float q0 = 0; q0 < pi2; q0 += 0.025f)
    {
        for (float q1 = 0; q1 < pi2; q1 += 0.025f)
        {
            Config qTest;
            qTest(0) = q0;
            qTest(1) = q1;
            float d = GetDist(qTest, c);
            if (d < 2.0f)
            {
                trueRobot.SetQ(qTest);
                trueRobot.Update();
                trueRobot.Draw();
                c.manifold.push_back(qTest);
            }
        }
    }
    */
    trueRobot.SetQ(q);
    trueRobot.Update();
}
