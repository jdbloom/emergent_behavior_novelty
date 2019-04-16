#include "emergent_behavior.h"
#include "buzz/buzzvm.h"

static CRange<Real> STIMULUS_RANGE(0.0, 1000.0);

/****************************************/
/****************************************/

/**
 * Functor to get data from the robots
 */
struct GetRobotData : public CBuzzLoopFunctions::COperation {

   /** Constructor */
   GetRobotData(int t) : temp(t) {}

   /** The action happens here */
   virtual void operator()(const std::string str_robot_id,
                           buzzvm_t t_vm) {

      buzzobj_t tCurX = BuzzGet(t_vm, "cur_x");
      if(!buzzobj_isfloat(tCurX)) {
         LOGERR << str_robot_id << ": variable 'cur_x' has wrong type " << buzztype_desc[tCurX->o.type] << std::endl;
         return;
      }

      float fCurX = buzzobj_getfloat(tCurX);

      buzzobj_t tCurY = BuzzGet(t_vm, "cur_y");
      if(!buzzobj_isfloat(tCurY)) {
         LOGERR << str_robot_id << ": variable 'cur_y' has wrong type " << buzztype_desc[tCurY->o.type] << std::endl;
         return;
      }

      float fCurY = buzzobj_getfloat(tCurY);

      buzzobj_t tCurZ = BuzzGet(t_vm, "cur_z");
      if(!buzzobj_isfloat(tCurZ)) {
         LOGERR << str_robot_id << ": variable 'cur_z' has wrong type " << buzztype_desc[tCurZ->o.type] << std::endl;
         return;
      }

      float fCurZ = buzzobj_getfloat(tCurZ);

      buzzobj_t tCurS = BuzzGet(t_vm, "cur_s");
      if(!buzzobj_isint(tCurS)) {
         LOGERR << str_robot_id << ": variable 'cur_s' has wrong type " << buzztype_desc[tCurS->o.type] << std::endl;
         return;
      }

      int iCurS = buzzobj_getint(tCurS);

      buzzobj_t tCurR = BuzzGet(t_vm, "cur_r");
      if(!buzzobj_isint(tCurR)) {
         LOGERR << str_robot_id << ": variable 'cur_r' has wrong type " << buzztype_desc[tCurR->o.type] << std::endl;
         return;
      }

      int iCurR = buzzobj_getint(tCurR);

      m_vecRobotX[t_vm->robot] = fCurX;
      m_vecRobotY[t_vm->robot] = fCurY;
      m_vecRobotZ[t_vm->robot] = fCurZ;
      m_vecRobotState[t_vm->robot] = iCurS;
      m_vecRobotReading[t_vm->robot] = iCurR;
      
   }

   int temp;

   std::map<int, float> m_vecRobotX;
   std::map<int, float> m_vecRobotY;
   std::map<int, float> m_vecRobotZ;
   std::map<int, int> m_vecRobotState;
   std::map<int, int> m_vecRobotReading;

};

/****************************************/
/****************************************/

// /**
//  * Functor to put the stimulus in the Buzz VMs.
//  */
// struct PutStimuli : public CBuzzLoopFunctions::COperation {

//    /** Constructor */
//    PutStimuli(const std::vector<float>& vec_stimuli) : m_vecStimuli(vec_stimuli) {}
   
//    /** The action happens here */
//    virtual void operator()(const std::string str_robot_id,
//                            buzzvm_t t_vm) {
//       /* Set the values of the table 'stimulus' in the Buzz VM */
//       BuzzTableOpen(t_vm, "stimulus");
//       for(int i = 0; i < m_vecStimuli.size(); ++i) {
//          BuzzTablePut(t_vm, i, m_vecStimuli[i]);
//       }
//       BuzzTableClose(t_vm);
//    }

//    /** Calculated stimuli */
//    const std::vector<float>& m_vecStimuli;
// };

/****************************************/
/****************************************/

void CEmergentBehavoir::Init(TConfigurationNode& t_tree) {
   /* Call parent Init() */
   CBuzzLoopFunctions::Init(t_tree);
   /* Parse XML tree */
   GetNodeAttribute(t_tree, "outfile", m_strOutFile);
   /* Open the output file */
   m_cOutFile.open(m_strOutFile.c_str(),
                   std::ofstream::out | std::ofstream::trunc);
}

/****************************************/
/****************************************/

void CEmergentBehavoir::Reset() {
   /* Reset the output file */
   m_cOutFile.open(m_strOutFile.c_str(),
                   std::ofstream::out | std::ofstream::trunc);

}

/****************************************/
/****************************************/

void CEmergentBehavoir::Destroy() {
   m_cOutFile.close();
}

/****************************************/
/****************************************/

void CEmergentBehavoir::PostStep() {
   char delim = ','
   /* Get robot data */
   GetRobotData cGetRobotData(1);
   BuzzForeachVM(cGetRobotData);
   
   /* Flush data to the output file */
   for(int i = 0; i < GetNumRobots(); ++i) {
      m_cOutFile << GetSpace().GetSimulationClock() << delim
      << i << delim
      << cGetRobotData.m_vecRobotX[i] << delim
      << cGetRobotData.m_vecRobotY[i] << delim
      << cGetRobotData.m_vecRobotZ[i] << delim
      << cGetRobotData.m_vecRobotState[i] << delim
      << cGetRobotData.m_vecRobotReading[i] << std::endl;
   }
}

/****************************************/
/****************************************/

bool CEmergentBehavoir::IsExperimentFinished() {
   /* Feel free to try out custom ending conditions */
   return false;
}

/****************************************/
/****************************************/

int CEmergentBehavoir::GetNumRobots() const {
   return m_mapBuzzVMs.size();
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CEmergentBehavoir, "emergent_behavior");
