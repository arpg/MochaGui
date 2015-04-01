#include "MochaGui/PlannerGui.h"

/////////////////////////////////////////////////////////////////////////////////////////
PlannerGui::PlannerGui():
    m_RenderState(pangolin::ProjectionMatrix(WINDOW_WIDTH,WINDOW_HEIGHT,420,420,WINDOW_WIDTH/2,WINDOW_HEIGHT/2,0.1,1000), pangolin::ModelViewLookAt(2,2,-3,0,0,0,pangolin::AxisNegZ)),
    m_RenderState2d(pangolin::ProjectionMatrixOrthographic(0,1,0,1,0.0,100)),
    m_RenderStateWidget(pangolin::ProjectionMatrixOrthographic(0, (double)UI_PANEL_WIDTH/(double)WINDOW_WIDTH,1.0-(double)UI_PANEL_HEIGHT/(double)WINDOW_HEIGHT,1.0,0.0,100)),
    m_pFollowCar(0),
    m_eViewType(eNeutral),
    m_nSelectedWaypoint(-1)
{
    int argc = 0;
    //glutInit(&argc,0);
    pangolin::CreateGlutWindowAndBind("Main",WINDOW_WIDTH,WINDOW_HEIGHT);
    glewInit();
}

/////////////////////////////////////////////////////////////////////////////////////////
PlannerGui::~PlannerGui()
{
    //delete all status texts
    for(size_t ii = 0 ; ii < m_vStatusLines.size() ; ii++){
        delete m_vStatusLines[ii];
    }

    for(size_t ii = 0 ; ii < m_vCars.size() ; ii++){
        delete m_vCars[ii];
    }

    for(size_t ii = 0 ; ii < m_vWaypoints.size() ; ii++){
        delete m_vWaypoints[ii];
    }



}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::Render()
{
    //boost::mutex::scoped_lock lock(m_DrawMutex);
    m_pView->ActivateScissorAndClear();

    if(m_pFollowCar != NULL){
        m_RenderState.Follow(OpenGlMatrix(m_pFollowCar->m_GLCar.GetPose4x4_po()));
    }

    //TODO: remove this and debug the GL problem that's occuring
    glGetError();

    // Swap frames and Process Events
    pangolin::FinishGlutFrame();

    //handle waypoints

    for(size_t ii = 0 ; ii < m_vWaypoints.size() ; ii++){
        if(m_vWaypoints[ii]->m_Waypoint.m_bPendingActive == true){
            m_nSelectedWaypoint = ii;
            m_vWaypoints[ii]->m_Waypoint.m_bPendingActive = false;
            m_vWaypoints[ii]->m_Waypoint.m_bActive = true;
            break;
        }
    }

    if(m_nSelectedWaypoint != -1){
        for(size_t ii = 0 ; ii < m_vWaypoints.size() ; ii++){
            if((int)ii != m_nSelectedWaypoint){
                m_vWaypoints[ii]->m_Waypoint.m_bActive = false;
                m_vWaypoints[ii]->m_Waypoint.m_bPendingActive = false;
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::Init(const std::string sTerrainMeshFileName, GLMesh* pMesh, const bool bViconCoords /* = false */)
{
    const aiScene *pScene = aiImportFile( sTerrainMeshFileName.c_str(), aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_JoinIdenticalVertices | aiProcess_OptimizeMeshes | aiProcess_FindInvalidData | aiProcess_FixInfacingNormals );
    std::cout << aiGetErrorString() << std::endl;

    if(bViconCoords){
        pScene->mRootNode->mTransformation = aiMatrix4x4(1,0,0,0,
                                                         0,-1,0,0,
                                                         0,0,-1,0,
                                                         0,0,0,1);
    }
    pMesh->Init(pScene);
    Init(pMesh);
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::Init(SceneGraph::GLObject* pTerrain)
{
    m_pTerrain = pTerrain;

    m_SceneGraph.ApplyPreferredGlSettings();
    m_SceneGraph2d.ApplyPreferredGlSettings();
    m_SceneGraphWidgets.ApplyPreferredGlSettings();
    glClearColor(0,0,0,0);

    //add the lights
    m_pLight = new SceneGraph::GLShadowLight(100,100,-100,1024,1024);
    m_pStaticLight = new SceneGraph::GLShadowLight(100,100,-100,4096,4096);
    m_pLight->SetShadowsEnabled(false);
    m_pStaticLight->SetShadowsEnabled(false);
    m_pLight->AddShadowReceiver(m_pTerrain);
    m_pStaticLight->AddShadowCasterAndReceiver(m_pTerrain);

    CheckForGLErrors();

    m_pStaticLight->SetAmbient(Eigen::Vector4f(0.1,0.1,0.1,1.0));
    m_pStaticLight->SetDiffuse(Eigen::Vector4f(0.4,0.4,0.4,1.0));
    m_pLight->SetAmbient(Eigen::Vector4f(0.1,0.1,0.1,1.0));
    m_pLight->SetDiffuse(Eigen::Vector4f(0.4,0.4,0.4,1.0));
    m_SceneGraph.AddChild(m_pStaticLight);
    m_SceneGraph.AddChild(m_pLight);

    m_pView = &pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0, 1.0)
            .SetHandler(new SceneGraph::HandlerSceneGraph(m_SceneGraph,m_RenderState,pangolin::AxisNegZ,0.01f))
            .SetDrawFunction(SceneGraph::ActivateScissorClearDrawFunctor3d2d(m_SceneGraph,m_RenderState, m_SceneGraph2d, m_RenderState2d));

    m_pPanelView = &pangolin::CreateDisplay()
            .SetBounds(1.0-(double)UI_PANEL_HEIGHT/(double)WINDOW_HEIGHT,1.0,0, (double)UI_PANEL_WIDTH/(double)WINDOW_WIDTH)
            .SetHandler(new PlannerHandler(m_SceneGraphWidgets,m_RenderState2d,pangolin::AxisNegZ,0.01f,&m_vWidgetPanels))
            .SetDrawFunction(ActivateScissorBlendedDrawFunctor(m_SceneGraphWidgets,m_RenderState2d));

    pangolin::RegisterKeyPressCallback( 'v', std::bind(&PlannerGui::_CommandHandler, this, eChangeView) );
    pangolin::RegisterKeyPressCallback( '+', std::bind(&PlannerGui::_CommandHandler, this, eIncreaseWpVel) );
    pangolin::RegisterKeyPressCallback( '-', std::bind(&PlannerGui::_CommandHandler, this, eDecreaseWpVel) );
    pangolin::RegisterKeyPressCallback( 'S', [this] {this->m_pPanelView->Show(!this->m_pPanelView->IsShown()); } );

    _PopulateSceneGraph();

    CheckForGLErrors();
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::AddPanel(GLWidgetPanel* pPanel)
{
    m_vWidgetPanels.push_back(pPanel);
    pPanel->Init(UI_PANEL_WIDTH,UI_PANEL_HEIGHT);
    m_SceneGraphWidgets.AddChild(pPanel);
}

/////////////////////////////////////////////////////////////////////////////////////////
int PlannerGui::AddStatusLine(StatusLineLocation location)
{
    //first find the last status line in this location
    StatusLine *pLastStatus = 0;
    for(size_t ii = 0 ; ii < m_vStatusLines.size() ; ii++){
        if(m_vStatusLines[ii]->m_Location == location){
            pLastStatus = m_vStatusLines[ii];
        }
    }

    Eigen::Vector2d pos(0,0);
    //if the last status is null, initialize its position
    if(pLastStatus == NULL){
        switch(location){
            case eTopLeft:
                pos << 0.01,0.975;
                break;

            case eBottomLeft:
                pos << 0.01,0.025;
                break;
        }
    }else {
        pos = pLastStatus->m_dPos2D;
        switch(location){
            case eTopLeft:
                pos[1] -= 0.025;
                break;

            case eBottomLeft:
                pos[1] -= 0.025;
                break;
        }
    }

    //and now create a new status line
    m_vStatusLines.push_back(new StatusLine());

    StatusLine* statusLine = m_vStatusLines.back();
    statusLine->m_dPos2D = pos;
    statusLine->m_Location = location;
    statusLine->m_GLText.SetPosition(pos[0],pos[1]);

    //boost::mutex::scoped_lock lock(m_DrawMutex);
    m_SceneGraph2d.AddChild(&statusLine->m_GLText);

    return m_vStatusLines.size()-1;
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::SetStatusLineText(int id, std::string text)
{
    //boost::mutex::scoped_lock lock(m_DrawMutex);
    m_vStatusLines[id]->m_GLText.SetText(text);
}

/////////////////////////////////////////////////////////////////////////////////////////
int PlannerGui::AddCar(const double& nWheelbase, const double& nWidth)
{
    m_vCars.push_back(new Car());
    Car* pCar = m_vCars.back();

    pCar->m_GLCar.Init(eMesh);
    pCar->m_GLCar.SetCarScale(Eigen::Vector3d(nWheelbase,nWidth,nWheelbase));

    pCar->m_CarLineSegments.SetColor(GLColor(0.0f,0.0f,1.0f));

    //add this to the scenegraph
    //boost::mutex::scoped_lock lock(m_DrawMutex);

    m_SceneGraph.AddChild(&pCar->m_GLCar);
    m_pLight->AddShadowCasterAndReceiver(&pCar->m_GLCar);
    m_pStaticLight->AddShadowReceiver(&pCar->m_GLCar);
    for(size_t ii = 0 ; ii < pCar->m_GLCar.GetWheels().size() ; ii++) {
        m_SceneGraph.AddChild(pCar->m_GLCar.GetWheels()[ii]);
    }

    //add the trajectory to the scenegraph
    m_SceneGraph.AddChild(&pCar->m_CarLineSegments);

    m_SceneGraph.AddChild(&pCar->m_Axis);

    return m_vCars.size()-1;
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::SetCarState(const int &id, const VehicleState &state, bool bAddToTrajectory /* = false */)
{
    Car* pCar = m_vCars[id];
    std::unique_lock<std::mutex> lock(*pCar, std::try_to_lock);

    Sophus::SE3d state_aug = state.m_dTwv;
    state_aug.translation() -= GetBasisVector(state_aug,2)*0.05;
    pCar->m_GLCar.SetPose(state_aug.matrix());

    Sophus::SE3d axisPose = state.m_dTwv;
    VehicleState::AlignWithVelocityVector(axisPose,state.m_dV);
    pCar->m_Axis.SetPose(axisPose.matrix());
    pCar->m_Axis.SetAxisSize(state.m_dV.norm());

    for(size_t ii = 0 ; ii <  state.m_vWheelStates.size() ; ii++) {
        Sophus::SE3d T = state.m_vWheelStates[ii];
        pCar->m_GLCar.SetRelativeWheelPose(ii,T);
    }

    //add this pose to the trajectory of the car if required
    if(bAddToTrajectory){
        pCar->m_CarLineSegments.AddVertex(state.m_dTwv.translation());
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::SetCarVisibility(const int &id, const bool &bVisible)
{
    Car* pCar = m_vCars[id];
    pCar->m_GLCar.SetVisible(bVisible);
    for(size_t ii = 0 ; ii < pCar->m_GLCar.GetWheels().size() ; ii++) {
        pCar->m_GLCar.GetWheels()[ii]->SetVisible(bVisible);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
int PlannerGui::AddWaypoint(const Eigen::Vector6d &pose, const double &velocity)
{
    m_vWaypoints.push_back(new Waypoint());
    Waypoint *pWaypoint = m_vWaypoints.back();

    pWaypoint->m_Waypoint.SetPose(pose);
    pWaypoint->m_Waypoint.SetVelocity(velocity);
    pWaypoint->m_Waypoint.SetDirty(true);

    //boost::mutex::scoped_lock lock(m_DrawMutex);
    m_SceneGraph.AddChild(&pWaypoint->m_Waypoint);

    return m_vWaypoints.size()-1;
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::ClearWaypoints()
{
    //remove all waypoints from the vector and from the scenegraph
    for(size_t ii = 0 ; ii < m_vWaypoints.size() ; ii++){
        m_SceneGraph.RemoveChild(&m_vWaypoints[ii]->m_Waypoint);
        delete m_vWaypoints[ii];
    }
    m_vWaypoints.clear();
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::SetWaypointDirtyFlag(bool bFlag)
{
    for(size_t ii = 0 ; ii < m_vWaypoints.size() ; ii++){
        m_vWaypoints[ii]->m_Waypoint.SetDirty(bFlag);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::_PopulateSceneGraph()
{


    //add the terrain
    m_SceneGraph.AddChild(m_pTerrain);

    //add a grid
    m_SceneGraph.AddChild(new SceneGraph::GLGrid());
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::_SetViewType(const PlannerViewType& eViewType)
{
    //get the car position
    Eigen::Vector4d target(0,0,0,1);
    Eigen::Vector4d nearSource(-0.5,0,-0.15,1);
    Eigen::Vector4d farSource(-1,0,-0.15,1);
    Eigen::Vector4d wheelSource(-0.5,-0.25,-0.15,1);

    //transform the sources based on where the car is
    Car *pCar = m_vCars[0];
    target = pCar->m_GLCar.GetPose4x4_po() * target;
    nearSource = pCar->m_GLCar.GetPose4x4_po() * nearSource;
    farSource = pCar->m_GLCar.GetPose4x4_po() * farSource;
    wheelSource = pCar->m_GLCar.GetPose4x4_po() * wheelSource;

    switch(eViewType){
        case eNeutral:
            SetFollowCar(-1);
            m_RenderState.Unfollow();
            break;

        case eFollowNear:
            //set the model view matrix
        m_RenderState.SetModelViewMatrix(pangolin::ModelViewLookAt(nearSource[0],nearSource[1],nearSource[2],target[0],target[1],target[2],pangolin::AxisNegZ));
            SetFollowCar(0);
            break;

        case eFollowFar:
            //set the model view matrix
            m_RenderState.SetModelViewMatrix(pangolin::ModelViewLookAt(farSource[0],farSource[1],farSource[2],target[0],target[1],target[2],pangolin::AxisNegZ));
            SetFollowCar(0);
            break;

        case eFollowWheel:
            //set the model view matrix
            m_RenderState.SetModelViewMatrix(pangolin::ModelViewLookAt(wheelSource[0],wheelSource[1],wheelSource[2],target[0],target[1],target[2],pangolin::AxisNegZ));
            SetFollowCar(0);
            break;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void PlannerGui::_CommandHandler(const PlannerGuiCommands& command)
{
    switch(command){
        case eChangeView:
            // advance the view type
            m_eViewType = m_eViewType == eFollowWheel ? eNeutral : (PlannerViewType)(m_eViewType+1);
            //and now set this view
            _SetViewType(m_eViewType);
            break;

        case eVideoToggle:
             //"mencoder &fifo_file -demuxer rawvideo  -rawvideo fpgs=20:w=$w:h=$h:format=rgb24 -o &output_file -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=12000"
            //create file with this script
            //chmod the properties to executable 77
            //create command string with script name + variables
            //pipe output to dev null (in the string)
            //use system command to call mencoder

            //hold space for openGL buffer
            //glPixelStorei(GL_U
            //glReadBuffer(GL_BACK)
            //glReadPixels
            break;

    case eIncreaseWpVel:
        if(m_nSelectedWaypoint != -1){
            m_vWaypoints[m_nSelectedWaypoint]->m_Waypoint.SetVelocity(m_vWaypoints[m_nSelectedWaypoint]->m_Waypoint.GetVelocity()*1.05);
        }
        break;

    case eDecreaseWpVel:
        if(m_nSelectedWaypoint != -1){
            m_vWaypoints[m_nSelectedWaypoint]->m_Waypoint.SetVelocity(m_vWaypoints[m_nSelectedWaypoint]->m_Waypoint.GetVelocity()*0.95);
        }
        break;
    }
}