#ifndef GLGUIPANEL_H
#define GLGUIPANEL_H

#include <CarPlanner/CarParameters.h>

#include <stdio.h>
#include <stdlib.h>
#include <iomanip>
#include "MochaGui/GLWidgetPanel.h"
#include "Eigen/Eigen"


class GLGuiPanel : public GLWidgetPanel {
public:

    GLGuiPanel(){
    }
    ~GLGuiPanel()
    {
        delete[] m_pControlTargetLabels[0];
        delete[] m_pControlTargetLabels[1];
        delete[] m_pControlTargetLabels;
    }

    void Init()
    {

        //create the control labels
        m_pControlTargetLabels = new char*[2];
        m_pControlTargetLabels[0] = new char[100];
        m_pControlTargetLabels[1] = new char[100];
        strcpy(m_pControlTargetLabels[0],"Simulation");
        strcpy(m_pControlTargetLabels[1],"Experiment");
    }


    virtual void DrawUI() {
        Eigen::IOFormat CleanFmt(2, 0, ", ", "\n" , "[" , "]");
        nv::Rect frameRect(0,0,200,0);
        m_Ui.begin();

            m_Ui.beginFrame(nv::GroupFlags_GrowDownFromLeft,m_Rect);
                m_Ui.beginFrame(nv::GroupFlags_GrowDownFromLeft,frameRect);
                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect, "Paused:");
                        m_Ui.doLabel(m_Rect, (*GetVar<bool*>("interface:Paused")) ? "Yes" : "No", 0,0.1,0.8,0.8);
                        m_Ui.doLabel(m_Rect, "FPS:");
                        std::stringstream oss;
                        oss << std::fixed << std::setprecision(2) << *GetVar<double*>("interface:Fps");
                        m_Ui.doLabel(m_Rect, oss.str().c_str(), 0, 0.1, 0.8, 0.8);
                        // clear the string stream
                        oss.str("");
                    m_Ui.endGroup();
                m_Ui.endFrame();
                // Filter parameters panel
                m_Ui.beginFrame(nv::GroupFlags_GrowDownFromLeft,frameRect);
                    m_Ui.doLabel(m_Rect, "Filter Parameters");
                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect,"Size:");
                        oss << *GetVar<int*>("fusion:FilterSize");
                        m_Ui.doLabel(m_Rect, oss.str().c_str(),0,0.1,0.8,0.8);
                        oss.str("");
                        m_Ui.doLabel(m_Rect,"RMSE:");
                        oss << *GetVar<double*>("fusion:RMSE");
                        m_Ui.doLabel(m_Rect, oss.str().c_str(),0,0.1,0.8,0.8 );
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect,"Freq: IMU:");
                        oss << *GetVar<double*>("fusion:ImuFreq");
                        m_Ui.doLabel(m_Rect, oss.str().c_str(),0,0.1,0.8,0.8);
                        oss.str("");
                        m_Ui.doLabel(m_Rect,"Localizer:");
                        oss << *GetVar<double*>("fusion:LocalizerFreq");
                        m_Ui.doLabel(m_Rect, oss.str().c_str(),0,0.1,0.8,0.8);
                        oss.str("");
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect,"V:");
                        oss << *GetVar<double*>("fusion:Vel");
                        m_Ui.doLabel(m_Rect, oss.str().c_str(),0,0.1,0.8,0.8);
                        oss.str("");
                        m_Ui.doLabel(m_Rect,"X:");
                        oss << GetVar<Eigen::Vector3d*>("fusion:Pos")->transpose().format(CleanFmt);
                        m_Ui.doLabel(m_Rect, oss.str().c_str(),0,0.1,0.8,0.8);
                        oss.str("");
                    m_Ui.endGroup();
                m_Ui.endFrame();

                // Control parameters panel

                m_Ui.beginFrame(nv::GroupFlags_GrowDownFromLeft,frameRect);
                    m_Ui.doLabel(m_Rect, "Control Parameters");
                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doCheckButton(m_Rect,"Control On",GetVar<bool*>("control:ControlOn"));
                        m_Ui.doLabel(m_Rect, "Target:");
                        m_Ui.doComboBox(m_Rect,2,(const char **)m_pControlTargetLabels,GetVar<int*>("control:ControlTarget"));
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect, "Max Plan T:");
                        m_Ui.doProgressBar(m_Rect,0.0,1.0,GetVar<float*>("control:MaxControlPlanTime"));
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect, "Lookahead T:");
                        m_Ui.doProgressBar(m_Rect,0.0,1.0,GetVar<float*>("control:LookaheadTime"));
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLabel(m_Rect, "Plans/S:");
                        //m_Ui.doLabel(m_Rect,(boost::format("%.2f") % (*GetVar<double*>("control:PlansPerS"))).str().c_str(), 0,0.1,0.8,0.8); //crh
                        m_Ui.doLabel(m_Rect, "V error:");
                        //m_Ui.doLabel(m_Rect,(boost::format("%.2f") % (*GetVar<double*>("control:VelocityError"))).str().c_str(), 0,0.1,0.8,0.8); //crh
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                    m_Ui.doLabel(m_Rect, "Accel:");
                    m_Ui.doProgressBar(m_Rect,0.0,500.0,GetVar<double*>("control:Accel"));
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                    m_Ui.doLabel(m_Rect, "Steer:");
                    m_Ui.doProgressBar(m_Rect,0.0,500.0,GetVar<double*>("control:Steering"));
                    m_Ui.endGroup();

                m_Ui.endFrame();

                // planner parameters panel

                m_Ui.beginFrame(nv::GroupFlags_GrowDownFromLeft,frameRect);
                    m_Ui.doLabel(m_Rect, "Planner Parameters");
                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doCheckButton(m_Rect,"Planner On",GetVar<bool*>("planner:PlannerOn"));
                        m_Ui.doCheckButton(m_Rect,"Sim On",GetVar<bool*>("planner:SimulatePath"));
                    m_Ui.endGroup();
                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doCheckButton(m_Rect,"Cont. Planning",GetVar<bool*>("planner:ContinuousPlanning"));
                        m_Ui.doCheckButton(m_Rect,"Inertial Ctrl.",GetVar<bool*>("planner:InertialControl"));
                    m_Ui.endGroup();

                    nv::Rect comboRect(0,0,100,0);
                    nv::Rect buttonRect(0,0,50,0);
                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        //create the filenames array
                        std::vector<std::string>* vFileNames = GetVar<std::vector<std::string>*>("planner:FileNames");
                        const char** fileNames = new const char*[vFileNames->size()];
                        for(size_t ii = 0; ii < vFileNames->size() ; ii++){
                            fileNames[ii] = vFileNames->at(ii).c_str();
                        }

                        //m_Ui.doComboBox(comboRect,vFileNames->size(),fileNames,GetVar<int*>("planner:SelectedFileName")); //crh
                        delete[] fileNames;
                        m_Ui.doButton(buttonRect,"Load",GetVar<bool*>("planner:LoadWaypoints"));
                    m_Ui.endGroup();

                    m_Ui.beginGroup(nv::GroupFlags_GrowRightFromTop);
                        m_Ui.doLineEdit(comboRect,GetVar<char*>("planner:SaveFileName"),256,GetVar<int*>("planner:SaveFileNameLen"));
                        m_Ui.doButton(buttonRect,"Save",GetVar<bool*>("planner:SaveWaypoints"));
                    m_Ui.endGroup();
                m_Ui.endFrame();

            m_Ui.endFrame();
        m_Ui.end();

    }

private:

    char** m_pControlTargetLabels;

};
#endif // GLGUIPANEL_H
