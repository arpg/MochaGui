#ifndef GLLEARNINGPANEL_H
#define GLLEARNINGPANEL_H

#include <stdio.h>
#include <stdlib.h>
#include <CarPlanner/CarParameters.h>
#include "GLWidgetPanel.h"


class GLLearningPanel : public GLWidgetPanel {
public:

    GLLearningPanel(){
        m_pParamText = NULL;
    }
    ~GLLearningPanel()
    {
        if(m_pParamText != NULL){
            for(int ii = 0 ; ii < m_nParams ;ii++){
                delete[] m_pParamText[ii];
            }

            delete[] m_pParamText;
        }
    }

    void Init()
    {
        m_pParams1 = GetVar<std::vector<RegressionParameter>*>("learning:LearningParams");
        m_pParams2 = GetVar<std::vector<RegressionParameter>*>("learning:DriveParams");

        m_nParams = m_pParams1->size();
        m_pParamText = new char *[m_nParams];
        for(int ii = 0 ; ii < m_nParams ;ii++){
            m_pParamText[ii] = new char[100];
        }
    }


    virtual void DrawUI() {
        m_Ui.begin(); {
            m_Ui.beginFrame(nv::GroupFlags_GrowDownFromLeft); {
                m_Ui.doCheckButton(m_Rect, "Enable learning", GetVar<bool*>("learning:LearnOn"));
                m_Ui.doCheckButton(m_Rect, "Playback", GetVar<bool*>("learning:Playback"));
                m_Ui.doCheckButton(m_Rect, "Process Model", GetVar<bool*>("learning:ProcessModelEnabled"));

                m_Ui.beginGroup(nv::GroupFlags_GrowLeftFromTop); {
                    m_Ui.doButton(m_Rect, "Refresh", GetVar<bool*>("learning:Refresh"));
                    m_Ui.doButton(m_Rect, "Regress", GetVar<bool*>("learning:Regress"));
                }
                m_Ui.endGroup();

                m_Ui.beginGroup(nv::GroupFlags_GrowLeftFromTop); {
                    m_Ui.doLabel(m_Rect, (boost::format("dt:%ds") % *GetVar<double*>("learning:dt")).str().c_str() );

                    nv::Rect leRect(0,0,50,0) ;
                    DoNumericLineEdit(leRect,"Samples: ",GetVar<int*>("learning:TotalRegressionSamples"));
                }
                m_Ui.endGroup();



                //show the number of collected samples
                m_Ui.beginGroup(nv::GroupFlags_GrowLeftFromTop); {
                    m_Ui.doLabel(m_Rect, "Collected :" );

                    float val = *GetVar<int*>("learning:CollectedRegressionSamples");
                    m_Ui.doProgressBar(m_Rect,0,*GetVar<int*>("learning:TotalRegressionSamples"),&val);
                }
                m_Ui.endGroup();

                int selected = -1;
                nv::Rect lRect(0,0,200,0) ;

                //draw the parameter list if necessary
                if(m_pParams1->size()!=0 && m_pParams2->size() !=0){
                    m_Ui.doLabel(m_Rect,"Parameters:");
                    //create the parameter list
                    for(int ii = 0 ; ii < m_nParams ;ii++){
                        sprintf(m_pParamText[ii],"%s \t %.2f \t %.2f" , m_pParams1->at(ii).m_sName.c_str(), m_pParams1->at(ii).m_dVal, m_pParams2->at(ii).m_dVal);
                    }
                    m_Ui.doListBox(lRect,m_pParams1->size(),(const char **)m_pParamText,&selected);
                }

                m_Ui.beginGroup(nv::GroupFlags_GrowLeftFromTop); {
                    m_Ui.doLabel(m_Rect, (boost::format("Rates IMU:%.2f") % *GetVar<double*>("learning:ImuRate")).str().c_str() );
                    m_Ui.doLabel(m_Rect, (boost::format(" Vicon:%.2f") % *GetVar<double*>("learning:ViconRate")).str().c_str() );

                }
                m_Ui.endGroup();

                m_Ui.doCheckButton(m_Rect, "MapSteering", GetVar<bool*>("learning:MapSteering"));                
            }


            m_Ui.endFrame();
        }
        m_Ui.end();

    }

private:
    int m_nParams;
    char **m_pParamText;
    std::vector<RegressionParameter>* m_pParams1;
    std::vector<RegressionParameter>* m_pParams2;
};


#endif // GLLEARNINGPANEL_H
