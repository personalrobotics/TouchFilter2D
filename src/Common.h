/*
 * Common.h
 *
 *  Created on: Sep 28, 2015
 *      Author: mklingen
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "ofMain.h"

inline float wrap(float x, const float& a, const float& b)
{
    while(x > b)
    {
        x -= (b - a);
    }

    while (x < a)
    {
        x += (b - a);
    }

    return x;
}

inline float TorusDist(ofVec2f a, ofVec2f b)
{
    const float pi = 3.14159;
    float x1 = wrap(a.x, 0, 2 * pi);
    float x2 = wrap(b.x, 0, 2 * pi);
    float y1 = wrap(a.y, 0, 2 * pi);
    float y2 = wrap(b.y, 0, 2 * pi);
    float w = 2 * pi;
    float h = 2 * pi;
    return sqrt(pow(fmin(fabs(x1 - x2), w - fabs(x1 - x2)), 2) + pow(fmin(fabs(y1 - y2), h - fabs(y1 - y2)), 2));
}

inline float TorusDist(ofVec3f a, ofVec3f b)
{
    const float pi = 3.14159;
    float x1 = wrap(a.x, 0, 2 * pi);
    float x2 = wrap(b.x, 0, 2 * pi);
    float y1 = wrap(a.y, 0, 2 * pi);
    float y2 = wrap(b.y, 0, 2 * pi);
    float z1 = wrap(a.z, 0, 2 * pi);
    float z2 = wrap(b.z, 0, 2 * pi);
    float w = 2 * pi;
    float h = 2 * pi;
    float d = 2 * pi;
    return sqrt(pow(fmin(fabs(x1 - x2), w - fabs(x1 - x2)), 2) +
                pow(fmin(fabs(y1 - y2), h - fabs(y1 - y2)), 2) +
                pow(fmin(fabs(z1 - z2), d - fabs(z1 - z2)), 2));
}

inline ofVec3f SampleBall()
{
    ofVec3f sample;
    do
    {
        sample.x = ofRandom(-1, 1);
        sample.y = ofRandom(-1, 1);
        sample.z = ofRandom(-1, 1);
    } while (sample.length() > 1);

    return sample;
}

inline ofVec2f SampleDisc()
{
    ofVec2f sample;
    do
    {
        sample.x = ofRandom(-1, 1);
        sample.y = ofRandom(-1, 1);
    } while (sample.length() > 1);

    return sample;
}


#endif /* COMMON_H_ */
