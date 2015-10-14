#ifndef NODE_H_
#define NODE_H_

namespace arm_slam
{
    class Node
    {
        public:
            Node() :
                parent(0x0),
                localTranslation(ofVec2f(0, 0)),
                globalTranslation(ofVec2f(0, 0)),
                localRotation(0.0f),
                globalRotation(0.0f)
            {

            }

            virtual ~Node()
            {

            }

            void UpdateRecursive()
            {
                if(!parent)
                {
                   globalTranslation = localTranslation;
                   globalRotation = localRotation;
                }
                else
                {
                    globalRotation = parent->globalRotation + localRotation;
                    globalTranslation = parent->globalTranslation + localTranslation.getRotatedRad(-globalRotation);
                }

                for (size_t i = 0; i < children.size(); i++)
                {
                    Node* node = children.at(i);
                    node->UpdateRecursive();
                }
            }

            virtual void DrawRecursive()
            {
                for (size_t i = 0; i < children.size(); i++)
                {
                    Node* node = children.at(i);
                    node->DrawRecursive();
                }
            }


            Node* parent;
            std::vector<Node*> children;
            ofVec2f localTranslation;
            ofVec2f globalTranslation;
            float localRotation;
            float globalRotation;
    };
}

#endif // NODE_H_ 
