#ifndef TSDF_H_
#define TSDF_H_

#include <vector>
#include "ofMain.h"
#include "World.h"
namespace arm_slam
{

    class TSDF
    {
        public:
            TSDF();
            virtual ~TSDF();

            void Initialize(int w, int h, float t)
            {
                width = w;
                height = h;
                truncation = t;
                dist.resize(width * height, truncation);
                weight.resize(width * height, 0);
            }

            void Initialize(World& world, float t, bool use_dist=false)
            {
                Initialize(world.data.width, world.data.height, t);

                for(int x = 0; x < width; x++)
                {
                    for(int y = 0; y < height; y++)
                    {
                        if(use_dist)
                        {
                            dist[GetIdx(x, y)] = world.GetDist(x, y);
                            weight[GetIdx(x, y)] = 200.0f;
                        }
                        else
                        {
                            dist[GetIdx(x, y)] = t;
                            weight[GetIdx(x, y)] = 0.0f;
                        }
                    }
                }
            }

            void SetColors(ofImage* img, ofImage* gradientMap)
            {
                ofColor color;
                for(int x = 0; x < width; x++)
                {
                    for(int y = 0; y < height; y++)
                    {
                        float d = dist[GetIdx(x, y)];
                        float w = weight[GetIdx(x, y)];

                        int xPos = (int)(fmin(fmax((((d + 128) / 255.0f) * gradientMap->width), 0.0f), gradientMap->width - 1));
                        color = gradientMap->getColor(xPos, 0);

                        /*
                        if(fabs(d) > 2)
                        {
                        color.setHue((d + truncation) / truncation * 64.0f);
                        color.setSaturation(255.0f);
                        color.setBrightness(255.0f);
                        }
                        else
                        {
                            color.r = 10;
                            color.g = 25;
                            color.b = 25;
                        }
                        */
                        color.a = w > 1e-5 ? 255 : 0;

                        if (fabs(d > 32.0f))
                        {
                            color.a = 0;
                        }

                        img->setColor(x, y, color);
                    }
                }
                img->update();
            }

            inline int GetIdx(int x, int y)
            {
                return x + y * width;
            }

            inline bool IsValid(int x, int y)
            {
                return x >= 0 && x < width && y >= 0 && y < height;
            }

            inline void SetWeight(int x, int y, float value)
            {
                weight[GetIdx(x, y)] = value;
            }

            inline void SetDist(int x, int y, float value)
            {
                dist[GetIdx(x, y)] = value;
            }

            inline float GetWeight(int x, int y)
            {
                if (IsValid(x, y))
                {
                    return weight[GetIdx(x, y)];
                }
                else
                {
                    return 0.0f;
                }
            }

            inline float GetDist(int x, int y)
            {
                if (IsValid(x, y))
                {
                    return dist[GetIdx(x, y)];
                }
                else
                {
                    return truncation;
                }
            }

            inline float sgn(float x)
            {
               return x > 0.0 ? 1.0f : -1.0f;
            }

            inline ofVec2f GetGradient(int x, int y)
            {
                float d0 = GetDist(x, y);
                float dxplus = GetDist(x + 1, y);
                float dyplus = GetDist(x, y + 1);
                float wxplus = GetWeight(x + 1, y);
                float wyplus = GetWeight(x, y + 1);
                float dxminus = GetDist(x - 1, y);
                float dyminus = GetDist(x, y - 1);
                float wxminus = GetWeight(x - 1, y);
                float wyminus = GetWeight(x, y - 1);

                if(wxplus > 2 && wyplus > 2 && wyminus > 2 && wxminus > 2)
                {
                    return ofVec2f((dxplus - dxminus) * 0.5f, (dyplus - dyminus) * 0.5f) * d0;
                }
                return ofVec2f(0, 0);
            }

            inline void FuseRayCloud(const ofVec2f& origin, const float& rotation, const std::vector<ofVec2f>& points, const std::vector<ofVec2f>& gradients)
            {
                for (size_t i =0; i < points.size(); i++)
                {
                    FuseRay(origin, points.at(i).getRotatedRad(-rotation) + origin, gradients.at(i).normalized());
                }
            }

            inline float GetWeight(float t)
            {
                float eps = truncation * 0.25f;
                return t < eps ? 1.0f : (truncation - t) / (truncation - eps);
            }

            inline void FuseRay(const ofVec2f& origin, const ofVec2f& end, const ofVec2f& normal)
            {
                ofVec2f p = origin;
                ofVec2f r = (end - origin);
                r.normalize();
                float dot = r.dot(normal);
                for (float t = -truncation; t < truncation; t++)
                {
                    p = end - r * t;

                    if(IsValid((int)p.x, (int)p.y))
                    {
                        FusePoint(p, t * dot, GetWeight(t * dot) * 0.1f);
                    }
                }
            }

            inline void FusePoint(const ofVec2f pos, float dist, float weight)
            {
                float oldSDF = GetDist((int)pos.x, (int)pos.y);
                float oldWeight = GetWeight((int)pos.x, (int)pos.y);
                float newDist = (oldWeight * oldSDF + weight * dist) / (weight + oldWeight);
                SetDist((int)pos.x, (int)pos.y, newDist);
                SetWeight((int)pos.x, (int)pos.y, oldWeight + weight);
            }

            inline void ComputeError(arm_slam::World& world, float& classificationError, float& distError)
            {
                distError = 0;
                classificationError = 0;
                size_t num = 0;
                size_t numIncorrect = 0;
                for (int x = 0; x < width; x++)
                {
                    for (int y = 0; y < height; y++)
                    {
                        float weight = GetWeight(x, y);

                        if (weight > 0)
                        {
                            num++;
                            float dist = GetDist(x, y);
                            float wDist = world.GetDist(x, y);

                            bool myClass = dist < 0;
                            bool worldClass = wDist < 0;
                            distError += pow(dist - wDist, 2);

                            if (myClass != worldClass)
                            {
                                numIncorrect++;
                            }
                        }
                    }
                }

                if (num > 0)
                {
                    //distError /= num;
                    classificationError = (float)numIncorrect / (float)num;
                }
            }

            float truncation;
            std::vector<float> dist;
            std::vector<float> weight;
            int width;
            int height;

    };

}

#endif // TSDF_H_ 
