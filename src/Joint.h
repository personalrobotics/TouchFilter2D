#ifndef JOINT_H_
#define JOINT_H_

#include "Node.h"

namespace arm_slam
{
    class Joint : public Node
    {
        public:
            Joint() : Node(), q(0)
            {

            }
            Joint(Node* _parent) : Node(), q(0)
            {
                parent = _parent;
                parent->children.push_back(this);
            }

            virtual ~Joint()
            {

            }

            void Update()
            {
                localRotation = q;
            }

            float q;
    };
}

#endif // JOINT_H_ 
