#ifndef OF_APP_H
#define OF_APP_H

#include "ofMain.h"

#include "ofxVideoRecorder.h"
#include "TouchFilter.h"
#include "AnalyticFilter.h"
#include "SimpleFilter.h"

class ofApp: public ofBaseApp
{
    public:

        enum Mode
        {
            Analytic,
            TSDF,
            Simple
        };

        void setup();
        void update();
        void draw();

        void keyPressed(int key);
        void keyReleased(int key);
        void mouseMoved(int x, int y);
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
        void windowResized(int w, int h);
        void dragEvent(ofDragInfo dragInfo);
        void gotMessage(ofMessage msg);
        void exit();

        Mode mode;

        AnalyticFilter* aFilter;
        arm_slam::TouchFilter* tFilter;
        SimpleFilter* sFilter;

        bool hasStarted;
        ofRectangle button0;
        ofRectangle button1;
        ofRectangle button2;
        ofTrueTypeFont textFont;
};
#endif
