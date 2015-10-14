#include "ofApp.h"
#include <fstream>
#include "Definitions.h"
#include "SimpleFilter.h"

//--------------------------------------------------------------
void ofApp::setup()
{
    ofSetFrameRate(60);
    mode = Simple;

    textFont.loadFont("arial.ttf", 8);

    hasStarted = false;
    button0 = ofRectangle(SCREEN_WIDTH / 2 - 64, 50, 128, 32);
    button1 = ofRectangle(SCREEN_WIDTH / 2 - 64, 90, 128, 32);
    button2 = ofRectangle(SCREEN_WIDTH / 2 - 64, 130, 128, 32);

    tFilter = new arm_slam::TouchFilter();
    tFilter->app = this;

    sFilter = new SimpleFilter();
    sFilter->app = this;

    aFilter = new AnalyticFilter();
    aFilter->app = this;
}

void ofApp::exit()
{

}

//--------------------------------------------------------------
void ofApp::update()
{
    if (hasStarted)
    {
        switch (mode)
        {
            case Analytic:
                aFilter->Update();
                break;
            case Simple:
                sFilter->Update();
                break;
            case TSDF:
                tFilter->Update();
                break;
        };
    }
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofClear(255);

    if (hasStarted)
    {
        switch (mode)
        {
            case Analytic:
                aFilter->Draw();
                break;
            case Simple:
                sFilter->Draw();
                break;
            case TSDF:
                tFilter->Draw();
                break;
        };
    }
    else
    {
        if (button0.inside(mouseX, mouseY))
        {
            ofSetColor(0);
            ofRect(button0);
            ofSetColor(255);
            textFont.drawString("TSDF", button0.x + 32, button0.y + 16);
        }
        else
        {
            ofSetColor(255);
            ofRect(button0);
            ofSetColor(0);
            textFont.drawString("TSDF", button0.x + 32, button0.y + 16);
        }

        if (button1.inside(mouseX, mouseY))
        {
            ofSetColor(0);
            ofRect(button1);
            ofSetColor(255);
            textFont.drawString("IK", button1.x + 32, button1.y + 16);
        }
        else
        {
            ofSetColor(255);
            ofRect(button1);
            ofSetColor(0);
            textFont.drawString("IK", button1.x + 32, button1.y + 16);
        }

        if (button2.inside(mouseX, mouseY))
        {
            ofSetColor(0);
            ofRect(button2);
            ofSetColor(255);
            textFont.drawString("Simple", button2.x + 32, button2.y + 16);
        }
        else
        {
            ofSetColor(255);
            ofRect(button2);
            ofSetColor(0);
            textFont.drawString("Simple", button2.x + 32, button2.y + 16);
        }
    }
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    switch (mode)
    {
        case Analytic:
            break;
        case Simple:
            sFilter->Pressed(key);
            break;
        case TSDF:
            break;
    };
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
    switch (mode)
    {
        case Analytic:
            break;
        case Simple:
            sFilter->Released(key);
            break;
        case TSDF:
            break;
    };
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
    if (!hasStarted)
    {
        if (button0.inside(x, y))
        {
            mode = TSDF;
            hasStarted = true;
        }
        else if (button1.inside(x, y))
        {
            mode = Analytic;
            hasStarted = true;
        }
        else if (button2.inside(x, y))
        {
            mode = Simple;
            hasStarted = true;
        }
    }

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{

}
