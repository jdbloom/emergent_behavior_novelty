#ifndef MPGA_EMERGENT_BEHAVIOR_LOOP_FUNCTIONS_H
#define MPGA_EMERGENT_BEHAVIOR_LOOP_FUNCTIONS_H

/* The Buzz controller */
#include <buzz/argos/buzz_loop_functions.h>

/* ARGoS-related headers */
#include <argos3/core/utility/math/rng.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_entity.h>
#include <argos3/plugins/robots/kheperaiv/simulator/kheperaiv_measures.h>

#include <loop_functions/mpga_loop_functions.h>

/****************************************/
/****************************************/

/*
 * The size of the genome.
 *
 * The genome is the set of possible wheel speeds and state changes. There are two wheels speeds (left, right)
 * and one internal state per combination of sensor readings and current state. Since there are 3 possible readings,
 * and 2 possible states, the number of weights is therefore:
 *
 * W = (St' + Sp) * (R * St) = (1 + 2) * (3 * 2) = 18
 *
 * where:
 *   W   = number of weights
 *   R   = number of readings
 *   St  = number of states
 *   St' = number of new states
 *   Sp  = number of speeds
 */
static const size_t GENOME_SIZE = 18;

const Real KHEPERAIV_BASE_RADIUS    = 0.0704;
static const std::string KH_CONTROLLER      = "ebc";
static const Real        KH_COMMRANGE       = 10. ;
static const UInt32      KH_DATASIZE        = 300;
static const Real        KH_INIT_DISTANCE   = 2 * KHEPERAIV_BASE_RADIUS * 0.01;

/****************************************/
/****************************************/

using namespace argos;

class CMPGAEmergentBehaviorLoopFunctions : public CMPGALoopFunctions {

public:

    CMPGAEmergentBehaviorLoopFunctions();

    virtual ~CMPGAEmergentBehaviorLoopFunctions();

    virtual void Init(TConfigurationNode &t_node);

    virtual void Reset();

    virtual void PreStep();

    /* Configures the robot controller from the genome */
    virtual void ConfigureFromGenome(const Real *pf_genome);

    /* Calculates the performance of the robot in a trial */
    virtual Real Score();

private:

    void CreateRobots(UInt32 un_robots);

    void printErr(std::string in);

    /* The initial setup of a trial */
    struct SInitSetup {
        CVector3 Position;
        CQuaternion Orientation;
    };

    std::vector<SInitSetup> m_vecInitSetup;
    std::vector<CKheperaIVEntity*> m_vecKheperas;
    std::vector<CBuzzController*> m_vecControllers;
    std::vector<Real> m_pfControllerParams;
    CRandom::CRNG *m_pcRNG;


};

#endif
