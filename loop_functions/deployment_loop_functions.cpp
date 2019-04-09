#include "deployment_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <sstream>
#include <unistd.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_measures.h>

/****************************************/
/****************************************/

CDeploymentLoopFunctions::CDeploymentLoopFunctions() {
    m_tStepInfo.m_cCentroid = CVector3::ZERO;
    m_mapRobotIndex.clear();
}

/****************************************/
/****************************************/

void CDeploymentLoopFunctions::Init(TConfigurationNode& t_node) {
  std::string strBuffer;
  /* Create the random number generator */
  m_pcRNG = CRandom::CreateRNG("argos");
  /* Parse 'deployment' node */
  /* Create file name with rhoT_T_alpha_rhoR_R_seed*/ 
   TConfigurationNode& tDeployment = GetNode(t_node, "deployment");
  /* Task radius rhoT*/
  GetNodeAttribute(tDeployment, "task_radius", strBuffer);
  m_fTaskRadius = std::atof(strBuffer.c_str());
  m_strOutput = strBuffer;
  /* Number of workers(tasks) */
  GetNodeAttribute(tDeployment, "nb_workers", strBuffer);
  m_unNumTasks = std::atoi(strBuffer.c_str());
  m_strOutput = m_strOutput + "_" + strBuffer;
  /* Redundancy factor */
  GetNodeAttribute(tDeployment, "redundancy_factor", strBuffer);
  m_fRedundancy = std::atof(strBuffer.c_str());
  m_strOutput = m_strOutput + "_" + strBuffer;
  /* Is Running on Vicon */
  try{
      GetNodeAttribute(tDeployment, "is_vicon", strBuffer);
      m_bIsVicon = (std::atof(strBuffer.c_str()))?true:false;
  }
  catch(...) {
      m_bIsVicon = false;
  }
  /* Max Speed */
  Real m_fMaxRobotSpeed;
  TConfigurationNode& tConfRoot = GetSimulator().GetConfigurationRoot();
  TConfigurationNode& tControllers = GetNode(tConfRoot, "controllers");
  TConfigurationNode& tBuzzController = GetNode(tControllers, "buzz_controller_kheperaiv");
  TConfigurationNode& tParams = GetNode(tBuzzController, "params");
  TConfigurationNode& tWheelTurning = GetNode(tParams, "wheel_turning");
  GetNodeAttribute(tWheelTurning, "max_speed", m_fMaxRobotSpeed);
  /*
    * Define maximum duration of simulation as 10 times the minimum
    * time a robot takes to from the center of the task circle to any
    * task.
    */
  m_unMaxTimeSteps = 10 * m_fTaskRadius / (m_fMaxRobotSpeed / 100.0) * CPhysicsEngine::GetInverseSimulationClockTick();

  /* Robot distribution radius rhoR */
  m_strOutput = m_strOutput + "_" + ToString(m_fTaskRadius/3);
  /* Number of robots R */
  m_unNumRobots = m_unNumTasks * (m_fRedundancy * m_fTaskRadius /(TARGET/100.) + 1);

  if(!m_bIsVicon)
      SecondaryInit();

//  m_strOutput = m_strOutput + "_" + ToString(m_unNumRobots);
//  /* Random seed */
//  m_strOutput = m_strOutput + "_" + ToString(CSimulator::GetInstance().GetRandomSeed());

//  m_strOutput = m_strOutput + ".dat";

//  /* Open the file, erasing its contents (trunc) or appending (app) */
//  m_cOutputR.open("R_" + m_strOutput , std::ios_base::trunc | std::ios_base::out);
//  m_cOutputS.open("S_" + m_strOutput , std::ios_base::trunc | std::ios_base::out);
//  m_cOutputT.open("T_" + m_strOutput , std::ios_base::trunc | std::ios_base::out);

//  m_cOutputR << "timestep, robot_id, parent_id_old, parent_id_new, position_x, position_y, orientation_theta, state, substate, working?" << std::endl;
//  m_cOutputS << "timestep, estimated_centroid_x, estimated_centroid_y, intree_count, root" << std::endl;
//  m_cOutputT << "task_id, position_x, position_y, worker" << std::endl;

//  if(!m_bIsVicon){
//      CreateRobots();
//      PlaceCluster();
//  }

//  PlaceTasks();
//  SelectWorkers();
//  SelectRoot();



//  for (size_t i= 0; i < m_unNumTasks; ++i) {
//     m_cOutputT << i << ", " << m_tInitSetup.TaskPositions[i].GetX() <<
//      ", " << m_tInitSetup.TaskPositions[i].GetY() << ", " << m_tInitSetup.Workers[i] << std::endl;
//  }

//  UpdateParams();
  //DEBUG("Finished Init deployment loop function\n");
}

/****************************************/
/****************************************/

void CDeploymentLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

/*void CDeploymentLoopFunctions::Destroy() {

    // for (int i = 0; i < m_nNbRobots; ++i) {
    //    CEntity& entity = GetSpace().GetEntity("fb" + ToString(i));
    //    RemoveEntity(entity);
    // }
    m_vecWorkers.clear();
    //m_tStepInfo.m_tEdges.clear();
    ClearEdgeMap(m_tStepInfo.m_sEdges);
    m_tStepInfo.m_tHeadings.clear();
    //m_tTasks.clear();
    //m_tInitSetup.RobotPositions.clear();

   
    m_cOutputR.close();
    m_cOutputS.close();
    m_cOutputT.close();
}*/

/****************************************/
/****************************************/

void CDeploymentLoopFunctions::PreStep() {

}

/****************************************/
/****************************************/
void CDeploymentLoopFunctions::PostStep() {
    //DEBUG("Poststep\n");

//   m_tStepInfo.m_tEdges.clear();
   ClearEdgeMap(m_tStepInfo.m_sEdges);
   m_tStepInfo.m_tHeadings.clear();

   int nParent = -1;
   int nNewParent = -1;
   int nSubstate;
   int nClock = GetSpace().GetSimulationClock();
   Real fCentroidX, fCentroidY;
   int nTreeCount;
   for(int i = 0; i < m_pcKheperas.size() ; ++i) {
        nSubstate = -1;
       /* Get handle to footbot entity and controller */
        CKheperaIVEntity& cSon = *m_pcKheperas[i];
        CBuzzController& cController = *m_pcControllers[i];
        buzzvm_t tBuzzVM = cController.GetBuzzVM();
        /* Get state of current robot*/
        int nState = _GetGlobalInt("state",tBuzzVM);
        if(nState == 6) {
           nSubstate = _GetGlobalInt("call_state",tBuzzVM);
        }
        /* Get working status of current robot*/
        int nWorking = _GetGlobalInt("working",tBuzzVM);
        /* Get boolean variable indicating if root */
        int bRoot = _GetGlobalInt("root",tBuzzVM);
        /* Get boolean variable indicating if working */
        int bWorking = _GetGlobalInt("working",tBuzzVM);
        /* Get id of current robot */
        const std::string& strRobotId = cSon.GetId();
        //DEBUG("robot id: %s\n",strRobotId.c_str());
        /* Get absolute position of current robot */
        CVector3 cStart = cSon.GetEmbodiedEntity().GetOriginAnchor().Position;
        if(m_bIsVicon)
            cStart.SetZ(0.1);
        /* Get id and handle of selected parent in old tree */
        const char * pchTemp = "old_tree";
        nParent = _GetParentId(pchTemp, tBuzzVM);
        nParent = m_bIsVicon?m_mapRobotIndex.find(nParent)->second:nParent;

        if(nParent != -1) {

           CKheperaIVEntity& cParent = *m_pcKheperas[nParent];
        
           /* Get absolute position of parent robot */
           CVector3 cEnd = cParent.GetEmbodiedEntity().GetOriginAnchor().Position;
           if(m_bIsVicon)
               cEnd.SetZ(0.1);
//           m_tStepInfo.m_tEdges[make_pair(strRobotId, pchTemp)] = CRay3(cStart, cEnd);
           m_tStepInfo.m_sEdges.vecRobotId.push_back(strRobotId);
           m_tStepInfo.m_sEdges.vecTreeType.push_back(pchTemp);
           m_tStepInfo.m_sEdges.vecEdge.push_back(CRay3(cStart,cEnd));
           m_tStepInfo.m_sEdges.vecIsWorking.push_back(bWorking);
        }
        else {

//           m_tStepInfo.m_tEdges[make_pair(strRobotId, pchTemp)] = CRay3(cStart, cStart);
            m_tStepInfo.m_sEdges.vecRobotId.push_back(strRobotId);
            m_tStepInfo.m_sEdges.vecTreeType.push_back(pchTemp);
            m_tStepInfo.m_sEdges.vecEdge.push_back(CRay3(cStart,cStart));
            m_tStepInfo.m_sEdges.vecIsWorking.push_back(bWorking);
        }

        /* Get id and handle of selected parent in new tree */
        pchTemp = "new_tree";
        nNewParent = _GetParentId(pchTemp, tBuzzVM);
        nNewParent = m_bIsVicon?m_mapRobotIndex.find(nNewParent)->second:nNewParent;


        if(nNewParent != -1) {
           CKheperaIVEntity& cParent = *m_pcKheperas[nNewParent];
           /* Get absolute position of parent robot */
           CVector3 cEnd = cParent.GetEmbodiedEntity().GetOriginAnchor().Position;
           if(m_bIsVicon)
               cEnd.SetZ(0.1);
//           m_tStepInfo.m_tEdges[make_pair(strRobotId, pchTemp)] = CRay3(cStart, cEnd);
           m_tStepInfo.m_sEdges.vecRobotId.push_back(strRobotId);
           m_tStepInfo.m_sEdges.vecTreeType.push_back(pchTemp);
           m_tStepInfo.m_sEdges.vecEdge.push_back(CRay3(cStart,cEnd));
           m_tStepInfo.m_sEdges.vecIsWorking.push_back(bWorking);
        }
        if(_GetCentroid(tBuzzVM) != CVector3::ZERO) {
           m_tStepInfo.m_cCentroid = cStart + _GetCentroid(tBuzzVM).Rotate(\
            cSon.GetEmbodiedEntity().GetOriginAnchor().Orientation); 
        }

        m_tStepInfo.m_tHeadings.push_back(CRay3(cStart, cStart + _GetHeading(tBuzzVM).Rotate(\
          cSon.GetEmbodiedEntity().GetOriginAnchor().Orientation)));


        CRadians cXAngle, cYAngle, cZAngle;

        cSon.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);

        if (bRoot == 1) {
          nTreeCount = _GetGlobalInt("robot_count",tBuzzVM);
          //fCentroidX = _GetCentroid(tBuzzVM).GetX();
          //fCentroidY = _GetCentroid(tBuzzVM).GetY();
          fCentroidX = _GetGlobalFloat("accum_x",tBuzzVM)/nTreeCount/100;
          fCentroidY = _GetGlobalFloat("accum_y",tBuzzVM)/nTreeCount/100;
          // "timestep, estimated_centroid_x, estimated_centroid_y, intree_count"
          m_cOutputS << nClock << ", " << fCentroidX << ", " << fCentroidY << ", " << nTreeCount << ", " << i << std::endl;
        }

        m_cOutputR << nClock << ", " << FromString<int>(m_bIsVicon?strRobotId.substr(8):strRobotId.substr(2)) << ", " << nParent << ", " << nNewParent << ", " << cStart.GetX() << ", "
        << cStart.GetY() << ", " << cZAngle.GetValue() << ", " << nState << ", " << nSubstate << ", " << nWorking << std::endl;

   }
}




/****************************************/
/****************************************/

int CDeploymentLoopFunctions::_GetGlobalInt(const char* str_variable, buzzvm_t t_buzz_vm) {

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, str_variable, 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type == BUZZTYPE_INT){
    int nState = tVar->i.value;
    buzzvm_pop(t_buzz_vm);
    return nState;
   }
   else {
    buzzvm_pop(t_buzz_vm);
    return -1;
   }

}

Real CDeploymentLoopFunctions::_GetGlobalFloat(const char* str_variable, buzzvm_t t_buzz_vm) {

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, str_variable, 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type == BUZZTYPE_FLOAT){
    Real fVar = tVar->f.value;
    buzzvm_pop(t_buzz_vm);
    return fVar;
   }
   else {
    buzzvm_pop(t_buzz_vm);
    return -1;
   }

}

/****************************************/
/****************************************/

int CDeploymentLoopFunctions::_GetParentId(const char* str_variable, buzzvm_t t_buzz_vm) {

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, str_variable, 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type == BUZZTYPE_NIL){
    buzzvm_pop(t_buzz_vm);
    return -1;
   }

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "parent", 1));
   /* Load table variable value */
   buzzvm_tget(t_buzz_vm);
   if (tVar->o.type == BUZZTYPE_NIL){
    buzzvm_pop(t_buzz_vm);
    return -1;
   }

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "id", 1));
   /* Load table variable value */
   buzzvm_tget(t_buzz_vm);
   /* Get pointer to actual object */
   tVar = buzzvm_stack_at(t_buzz_vm, 1);
   /* Get value from the pointer*/
   UInt32 nParent = tVar->i.value;
   if (tVar->o.type == BUZZTYPE_INT) {
     /* Remove the var name from the stack */
     buzzvm_pop(t_buzz_vm);
     return nParent;
   }
   else {
     buzzvm_pop(t_buzz_vm);
     return -1;
   }
}

/****************************************/
/****************************************/

CVector3 CDeploymentLoopFunctions::_GetCentroid(buzzvm_t t_buzz_vm) {

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "centroid_x", 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   /* Get pointer to actual object */
   buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type != BUZZTYPE_FLOAT) {
    buzzvm_pop(t_buzz_vm);
    return CVector3::ZERO;
   }
   /* Get value from the pointer*/
   Real fCentroidX = tVar->f.value;
   /* Remove the var name from the stack */
   buzzvm_pop(t_buzz_vm);
   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "centroid_y", 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   /* Get pointer to actual object */
   tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type != BUZZTYPE_FLOAT) {
    buzzvm_pop(t_buzz_vm);
    return CVector3::ZERO;
   }
   /* Get value from the pointer*/
   Real fCentroidY = tVar->f.value;
   /* Remove the var name from the stack */
   buzzvm_pop(t_buzz_vm);
   
   return CVector3(fCentroidX/100, fCentroidY/100, 0.3);

}

/****************************************/
/****************************************/

CVector3 CDeploymentLoopFunctions::_GetHeading(buzzvm_t t_buzz_vm) {

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "goal", 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   /* Get pointer to actual object */
   buzzobj_t tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type == BUZZTYPE_NIL) {
    buzzvm_pop(t_buzz_vm);
    return CVector3::ZERO;
   }
   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "x", 1));
   /* Load table variable value */
   buzzvm_tget(t_buzz_vm);
   /* Get pointer to actual object */
   tVar= buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type != BUZZTYPE_FLOAT) {
    buzzvm_pop(t_buzz_vm);
    return CVector3::ZERO;
   }
   Real fGoalX = tVar->f.value;
   /* Remove the var name from the stack */
   buzzvm_pop(t_buzz_vm);

   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "goal", 1));
   /* Load global variable value */
   buzzvm_gload(t_buzz_vm);
   /* Push var name on the stack */
   buzzvm_pushs(t_buzz_vm, buzzvm_string_register(t_buzz_vm, "y", 1));
   /* Load table variable value */
   buzzvm_tget(t_buzz_vm);
   /* Get pointer to actual object */
   tVar = buzzvm_stack_at(t_buzz_vm, 1);
   if (tVar->o.type != BUZZTYPE_FLOAT) {
    buzzvm_pop(t_buzz_vm);
    return CVector3::ZERO;
   }
   Real fGoalY = tVar->f.value;
   /* Remove the var name from the stack */
   buzzvm_pop(t_buzz_vm);

   return CVector3(fGoalX/100, fGoalY/100, 0.0);

}


/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CDeploymentLoopFunctions, "deployment_loop_functions")

  // m_unSeed = CSimulator::GetInstance().GetRandomSeed();
  //UInt16 nCurrentId = FromString<UInt16>(strRobotId.substr(2));
  //LOG<<"Son\t"<<nCurrentId<<"\tParent\t"<<m_nParent<<std::endl;
  // TConfigurationNode& tControllers = GetNode(CSimulator::GetInstance().GetConfigurationRoot(), "controllers");
  // TConfigurationNode& tBuzzController = GetNode(tControllers, "buzz_controller_footbot");
  // TConfigurationNode& tSensors = GetNode(tBuzzController, "sensors");
  // TConfigurationNode& tRAB = GetNode(tSensors, "range_and_bearing");
  // GetNodeAttribute(tRAB,"rab_range", strBuffer);
  // m_strOutput = m_strOutput + "_" + strBuffer;
    //m_strOutput.c_str()
    //LOG<<<<std::endl;

    /* Open the file, erasing its contents (trunc) or appending (app) */ 
    //m_cOutput.open(m_strOutput.c_str(), std::ios_base::app | std::ios_base::out);

    //m_cOutput <<"Seed\t" <<  << std::endl;

    //std::time_t t = std::time(nullptr);
    //m_cOutput << "Experiment\t" << std::put_time(std::localtime(&t), "%c %Z") << " , , , , " << std::endl;