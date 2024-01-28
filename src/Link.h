#ifndef LINK_H_
#define LINK_H_

#include "Node.h"

namespace arm_slam
{
    class Link : public Node
    {
    public:
        Link() : Node()
        {
        }
        Link(Node *_parent, float length, ofColor c) : Node()
        {
            color = c;
            parent = _parent;
            parent->children.push_back(this);
            localTranslation = ofVec2f(length, 0);
        }

        virtual ~Link()
        {
        }

        virtual void DrawRecursive()
        {
            ofSetColor(color);
            ofSetLineWidth(10);
            if (parent == 0x0)
            {
                return;
            }
            else
            {
                // New open frameworks requires lines to be in 3D: https://openframeworks.cc/documentation/graphics/ofPolyline/
                ofVec3f parent_3f = ofVec3f(parent->globalTranslation.x, parent->globalTranslation.y, 0);
                ofVec3f self_3f = ofVec3f(globalTranslation.x, globalTranslation.y, 0);
                ofLine(parent_3f, self_3f);
            }
            Node::DrawRecursive();
        }

        ofColor color;
    };
}

#endif // LINK_H_
