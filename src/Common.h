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
    float x1 = a.x;
    float x2 = b.x;
    float y1 = a.y;
    float y2 = b.y;
    float w = pi;
    float h = pi;
    return sqrt(pow(fmin(fabs(x1 - x2), w - fabs(x1 - x2)), 2) + pow(fmin(fabs(y1 - y2), h - fabs(y1-y2)), 2));
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
