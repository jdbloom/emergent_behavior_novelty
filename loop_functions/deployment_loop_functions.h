#ifndef DEPLOYMENT_LOOP_FUNCTIONS_H
#define DEPLOYMENT_LOOP_FUNCTIONS_H

/* The controller */
#include <buzz/argos/buzz_controller.h>

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/core/simulator/loop_functions.h>

using namespace argos;

class CDeploymentLoopFunctions : public CLoopFunctions {

public:

   struct SEdgeMap{
       std::vector<std::string> vecRobotId;
       std::vector<std::string> vecTreeType;
       std::vector<CRay3> vecEdge;
       std::vector<int> vecIsWorking;
   };

   typedef std::vector<CRay3> THeadingVec;

public:

   CDeploymentLoopFunctions();
   virtual ~CDeploymentLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_node);
   virtual void Reset();
   virtual void PreStep();
   virtual void PostStep();


   inline void ClearEdgeMap(SEdgeMap &s_edges){
       s_edges.vecRobotId.clear();
       s_edges.vecTreeType.clear();
       s_edges.vecEdge.clear();
       s_edges.vecIsWorking.clear();
   }

private:

   Real _GetGlobalFloat(const char* str_variable, buzzvm_t t_buzz_vm);
   int _GetParentId(const char* str_variable, buzzvm_t t_buzz_vm);
   int _GetGlobalInt(const char* str_variable, buzzvm_t t_buzz_vm);
   CVector3 _GetCentroid(buzzvm_t t_buzz_vm);
   CVector3 _GetHeading(buzzvm_t t_buzz_vm);

private:

   /* Output file variables */
   std::string m_strOutput;
   std::ofstream m_cOutputR;
   std::ofstream m_cOutputS;
   std::ofstream m_cOutputT;
   
private:

   /* The initial setup */
   struct SInitSetup {
      std::vector<CVector3> RobotPositions;
      std::vector<CQuaternion> RobotOrientations;
      std::vector<CVector3> TaskPositions;
      std::vector<size_t> Workers;
      size_t Root;
   };

   /* The current step data*/
   struct SStepInfo {
//      TEdgeMap m_tEdges;
      SEdgeMap m_sEdges;
      THeadingVec m_tHeadings;
      CVector3 m_cCentroid;
   };

   SInitSetup m_tInitSetup;
   SStepInfo m_tStepInfo;

   std::vector<CKheperaIVEntity*> m_pcKheperas;
   std::vector<CBuzzController*> m_pcControllers;

   UInt32 m_unSeed;
   UInt32 m_unNumTasks;
   UInt32 m_unNumRobots;
   UInt32 m_unMaxTimeSteps;
   Real m_fTaskRadius;
   Real m_fRedundancy; // redundancy factor
   bool m_bIsVicon;

   //   <robot_id,vector_index>
   std::map<int,int> m_mapRobotIndex;

   CRandom::CRNG* m_pcRNG;

};

#endif