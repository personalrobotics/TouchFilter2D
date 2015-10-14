/*
 * SimpleFilter.cpp
 *
 *  Created on: Sep 22, 2015
 *      Author: mklingen
 */

#include "SimpleFilter.h"

SimpleFilter::SimpleFilter()
{
    cam.setDistance(25);
    cam.setNearClip(0.05);
    cam.setFarClip(50.0f);

    keys['w'] = false;
    keys['a'] = false;
    keys['s'] = false;
    keys['d'] = false;
    keys['q'] = false;
    keys['e'] = false;

    int numParticles = 1000;
    ofVec3f start = ofVec3f(0, 0, GetManifold(0, 0) + 1);
    truePos = start;
    for (size_t i = 0; i < numParticles; i++)
    {
        Particle p;
        p.weight = 1.0f;
        p.pos = start + SampleBall() * 3.0f;
        particles.push_back(p);
    }
}

SimpleFilter::~SimpleFilter()
{

}


void SimpleFilter::Pressed(int key)
{
    keys[key] = true;
}

void SimpleFilter::Released(int key)
{
    keys[key] = false;
}


void SimpleFilter::Update()
{
    bool moved = false;
    ofVec3f movement = ofVec3f(0, 0, 0);

    float moveSpeed = 0.1;

    if (keys['w'])
    {
        moved = true;
        movement = ofVec3f(1, 0, 0);
    }

    if (keys['s'])
    {
        moved = true;
        movement = ofVec3f(-1, 0, 0);
    }

    if (keys['a'])
    {
        moved = true;
        movement = ofVec3f(0, 1, 0);
    }

    if (keys['d'])
    {
        moved = true;
        movement = ofVec3f(0, -1, 0);
    }

    if (keys['q'])
    {
        moved = true;
        movement = ofVec3f(0, 0, -1);
    }

    if (keys['e'])
    {
        moved = true;
        movement = ofVec3f(0, 0, 1);
    }

    if (moved)
    {
        MoveParticles(moveSpeed * movement);
        if (!colliding)
        {
            WeightParticles();
            ResampleParticles();
        }
    }

}


void SimpleFilter::MoveParticles(ofVec3f movement)
{
    truePos += movement;

    size_t numParticles = particles.size();
    float z = GetManifold(truePos.x, truePos.y);
    bool prevColliding = colliding;
    if (truePos.z <= z)
    {
        colliding = true;
        truePos.z = z;
    }
    else
    {
        colliding = false;
    }
    ofVec3f minBounds = ofVec3f(999, 999, 999);
    ofVec3f maxBounds = ofVec3f(-999, -999, -999);
    ofVec3f fuzz = ofVec3f(1, 1, 1) * 0.1f;
    for (size_t i = 0; i < numParticles; i++)
    {
        particles.at(i).pos += movement;

        minBounds.x = fmin(particles.at(i).pos.x, minBounds.x);
        minBounds.y = fmin(particles.at(i).pos.y, minBounds.y);
        minBounds.z = fmin(particles.at(i).pos.z, minBounds.z);

        maxBounds.x = fmax(particles.at(i).pos.x, maxBounds.x);
        maxBounds.y = fmax(particles.at(i).pos.y, maxBounds.y);
        maxBounds.z = fmax(particles.at(i).pos.z, maxBounds.z);

        // projection
        /*
        if (colliding)
        {
            particles.at(i).pos.z = GetManifold(particles.at(i).pos.x, particles.at(i).pos.y);
        }
        */
    }

    if (colliding && (!prevColliding))
    {
        std::vector<Particle> newParticles;
        float sumWeights = 0.0f;
        for (size_t i = 0; i < numParticles; i++)
        {
            Particle p;
            //p.pos = UniformSampleManifold(particles.at(i).pos - fuzz, particles.at(i).pos + fuzz);
            //p.pos = UniformSampleManifold(ofVec3f(-10, -10, 0), ofVec3f(10, 10, 0));
            p.pos = UniformSampleManifold(minBounds - fuzz, maxBounds + fuzz);
            //Particle& closest = GetClosest(p.pos);
            //p.weight = particles.at(i).weight;
            p.weight = GetKernelDensity(p.pos);
            //p.weight = exp(-10 * truePos.distance(p.pos));
            sumWeights += p.weight;
            //printf("%f\n", p.weight);
            newParticles.push_back(p);
        }

        for (size_t i = 0; i < numParticles; i++)
        {
            newParticles.at(i).weight /= sumWeights;
        }
        particles = newParticles;
        ResampleParticles();
    }
    else if (colliding && prevColliding)
    {
        for (size_t i = 0; i < numParticles; i++)
        {
            particles.at(i).pos.z = GetManifold(particles.at(i).pos.x, particles.at(i).pos.y);
        }
    }

}


SimpleFilter::Particle& SimpleFilter::GetClosest(const ofVec3f& pos)
{
    float mindist = 9999;
    Particle* minParticle = NULL;
    for (size_t i = 0; i < particles.size(); i++)
    {
        Particle& p = particles.at(i);
        float d = p.pos.distance(pos);
        if (d < mindist)
        {
            mindist = d;
            minParticle = &p;
        }
    }

    return *minParticle;
}

ofVec3f SimpleFilter::UniformSampleManifold(ofVec3f minBounds, ofVec3f maxBounds)
{
    float x = ofRandom(minBounds.x, maxBounds.x);
    float y = ofRandom(minBounds.y, maxBounds.y);
    float z = GetManifold(x, y);
    return ofVec3f(x, y, z);
}

void SimpleFilter::Draw()
{
    ofClear(255);
    DrawPlot();
}

void SimpleFilter::ResampleParticles()
{
    WeightParticles();

    float ballSize = 0.0005;
    std::vector<ofVec3f> offsets;

    for (size_t i = 0; i < particles.size(); i++)
    {
        offsets.push_back(particles.at(i).pos);
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
        particles.at(i).pos = offsets.at(j) + SampleBall() * ballSize;

        t += Minv;
    }
}

float SimpleFilter::GetKernelDensity(const ofVec3f& pos)
{
    float weight = 0.0f;
    for (size_t i = 0; i < particles.size(); i++)
    {
        const Particle& p = particles.at(i);
        weight += exp(-20 * p.pos.distance(pos));
    }

    return weight;
}

void SimpleFilter::WeightParticles()
{
    if (colliding)
    {

    }
    else
    {
        float sumWeights = 0;
        for (size_t i = 0; i < particles.size(); i++)
        {
            ofVec3f pos = particles.at(i).pos;
            float z = GetManifold(pos.x, pos.y);

            if (pos.z >= z)
            {
                particles.at(i).weight = 1.0f;
            }
            else
            {
                particles.at(i).weight = exp(-(z - pos.z));
            }

            sumWeights += particles.at(i).weight;
        }

        for (size_t p = 0; p < particles.size(); p++)
        {
            particles.at(p).weight /= sumWeights;
        }
    }
}

void SimpleFilter::DrawPlot()
{
    cam.begin();
    ofSetColor(200, 200, 200);
    for (float x = -10; x < 10; x+=0.5)
    {
        for (float y = -10; y < 10; y+=0.5)
        {
            float z = GetManifold(x, y);
            float zx = GetManifold(x + 0.5, y);
            float zy = GetManifold(x, y + 0.5);
            ofLine(x, y, z, x + 0.5, y, zx);
            ofLine(x, y, z, x , y + 0.5, zy);
        }
    }

    if (colliding)
    {
        ofSetColor(255, 100, 100);
    }
    else
    {
        ofSetColor(100, 100, 100);
        ofVec3f proj = truePos;
        proj.z = GetManifold(proj.x, proj.y);
        ofLine(truePos, proj);
        ofCircle(proj, 0.05);

        ofSetColor(100, 255, 100);
    }
    ofDrawSphere(truePos, 0.08);

    if (colliding)
    {
        ofSetColor(255, 100, 100);
    }
    else
    {
        ofSetColor(100, 100, 255);
    }

    for (size_t i = 0; i < particles.size(); i++)
    {
        ofVec3f pos = particles.at(i).pos;
        ofDrawSphere(pos, fmax(fmin(0.0002 * particles.at(i).weight * particles.size(), 0.2), 0.01));
    }
    cam.end();
}

float SimpleFilter::GetManifold(float x, float y)
{
    return 3.0 * ofNoise(x * 0.1, y * 0.1);
}
