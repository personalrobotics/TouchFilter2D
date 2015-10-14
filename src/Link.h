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
            Link(Node* _parent, float length, ofColor c) : Node()
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
                if(parent == 0x0)
                {
                    return;
                }
                else
                {

                    ofLine(parent->globalTranslation, globalTranslation);
                }
                Node::DrawRecursive();
            }

            ofColor color;
    };
}

#endif // LINK_H_ 
