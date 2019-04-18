#include "mpga_emergent_behavior_loop_functions.h"
#include <buzz/argos/buzz_controller.h>

/****************************************/
/****************************************/

CMPGAEmergentBehaviorLoopFunctions::CMPGAEmergentBehaviorLoopFunctions() {
    /*
     * Create the random number generator
     */
    m_pcRNG = CRandom::CreateRNG("argos");
    m_pfControllerParams.resize(GENOME_SIZE);
}

/****************************************/
/****************************************/

CMPGAEmergentBehaviorLoopFunctions::~CMPGAEmergentBehaviorLoopFunctions() {
    m_pfControllerParams.clear();
}

/****************************************/
/****************************************/

/**
 * Functor to put the stimulus in the Buzz VMs.
 */
struct PutGenome : public CBuzzLoopFunctions::COperation {

    /** Constructor */
    explicit PutGenome(const std::vector<float>& vec_genome) : m_vecGenome(vec_genome) {}

    /** The action happens here */
    virtual void operator()(const std::string str_robot_id,
                            buzzvm_t t_vm) {
        /* Set the values of the table 'stimulus' in the Buzz VM */
        BuzzTableOpen(t_vm, "genome");
        for(int i = 0; i < m_vecGenome.size(); ++i) {
            BuzzTablePut(t_vm, i, m_vecGenome[i]);
        }
        BuzzTableClose(t_vm);
    }

    /** Genome */
    const std::vector<float>& m_vecGenome;
};


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
        if (!buzzobj_isfloat(tCurX)) {
            LOGERR << str_robot_id << ": variable 'cur_x' has wrong type " << buzztype_desc[tCurX->o.type] << std::endl;
            return;
        }

        float fCurX = buzzobj_getfloat(tCurX);

        buzzobj_t tCurY = BuzzGet(t_vm, "cur_y");
        if (!buzzobj_isfloat(tCurY)) {
            LOGERR << str_robot_id << ": variable 'cur_y' has wrong type " << buzztype_desc[tCurY->o.type] << std::endl;
            return;
        }

        float fCurY = buzzobj_getfloat(tCurY);

        buzzobj_t tCurZ = BuzzGet(t_vm, "cur_z");
        if (!buzzobj_isfloat(tCurZ)) {
            LOGERR << str_robot_id << ": variable 'cur_z' has wrong type " << buzztype_desc[tCurZ->o.type] << std::endl;
            return;
        }

        float fCurZ = buzzobj_getfloat(tCurZ);

        buzzobj_t tCurS = BuzzGet(t_vm, "cur_s");
        if (!buzzobj_isint(tCurS)) {
            LOGERR << str_robot_id << ": variable 'cur_s' has wrong type " << buzztype_desc[tCurS->o.type] << std::endl;
            return;
        }

        int iCurS = buzzobj_getint(tCurS);

        buzzobj_t tCurR = BuzzGet(t_vm, "cur_r");
        if (!buzzobj_isint(tCurR)) {
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


void CMPGAEmergentBehaviorLoopFunctions::Init(TConfigurationNode &t_node) {

    int iNumRobots;
    GetNodeAttribute(t_node, "outfile", iNumRobots);

    CreateRobots(iNumRobots);

    /*
     * Create the initial setup for each trial
     * The robot is placed 4.5 meters away from the light
     * (which is in the origin) at angles
     * { PI/12, 2*PI/12, 3*PI/12, 4*PI/12, 5*PI/12 }
     * wrt to the world reference.
     * Also, the rotation of the robot is chosen at random
     * from a uniform distribution.
     */
    CRadians cOrient;

    CRadians step = CRadians::TWO_PI / static_cast<Real>(iNumRobots);

    for (size_t i = 0; i < iNumRobots; ++i) {
        /* Set position */
        m_vecInitSetup[i].Position.FromSphericalCoords(
                4.5f,                                          // distance from origin
                CRadians::PI_OVER_TWO,                         // angle with Z axis
                static_cast<Real>(i) * step                    // rotation around Z
        );
        /* Set orientation */
        cOrient = m_pcRNG->Uniform(CRadians::UNSIGNED_RANGE);
        m_vecInitSetup[i].Orientation.FromEulerAngles(
                cOrient,        // rotation around Z
                CRadians::ZERO, // rotation around Y
                CRadians::ZERO  // rotation around X
        );
    }

    /*
     * Process trial information, if any
     */
    try {
        UInt32 unTrial;
        GetNodeAttribute(t_node, "trial", unTrial);
        SetTrial(unTrial);
        Reset();
    }
    catch (CARGoSException &ex) {}
}

/****************************************/
/****************************************/

void CMPGAEmergentBehaviorLoopFunctions::Reset() {
    /*
     * Move robot to the initial position corresponding to the current trial
     */
    for(size_t i = 0; i < m_vecKheperas.size(); i++){
        if(!MoveEntity(
                m_vecKheperas[i]->GetEmbodiedEntity(),        //Move this robot
                m_vecInitSetup[i].Position,          // with this position
                m_vecInitSetup[i].Orientation,       // with this orientation
                false                                         // this is not a check, so actually move the robot back
                )) {
            LOGERR << "Can't move robot kh(" << i << ") in <"
                   << m_vecInitSetup[i].Position
                   << ">, <"
                   << m_vecInitSetup[i].Orientation
                   << ">"
                   << std::endl;
        }
    }
}

/****************************************/
/****************************************/

void CMPGAEmergentBehaviorLoopFunctions::ConfigureFromGenome(const Real *pf_genome) {
    /* Copy the genes into the NN parameter buffer */
    for (size_t i = 0; i < GENOME_SIZE; ++i) {
        m_pfControllerParams[i] = pf_genome[i];
    }
    PutGenome cPutGenome(m_pfControllerParams);
    BuzzForeachVM(cPutGenome);
}

/****************************************/
/****************************************/

Real CMPGAEmergentBehaviorLoopFunctions::Score() {
    GetRobotData cGetRobotData(0);
    BuzzForeachVM(cGetRobotData);
    /* The performance is simply the distance of the robot to the origin */
    return 1.0;
//   m_pcFootBot->GetEmbodiedEntity().GetOriginAnchor().Position.Length();
}

/****************************************/
/****************************************/

void CMPGAEmergentBehaviorLoopFunctions::CreateRobots(UInt32 un_robots) {
    for(size_t i = 0; i < un_robots; ++i) {
        /* Create robot */
        CKheperaIVEntity* pcRobot = new CKheperaIVEntity(
                "kh" + ToString(i),
                KH_CONTROLLER,
                CVector3(-20 + i * KH_INIT_DISTANCE, 20, 0),
                CQuaternion(),
                KH_COMMRANGE,
                KH_DATASIZE);
        /* Add it to the simulation */
        AddEntity(*pcRobot);
        /* Add it to the internal lists */
        m_vecKheperas.push_back(pcRobot);
        m_vecControllers.push_back(
                &dynamic_cast<CBuzzController&>(
                        pcRobot->GetControllableEntity().GetController()));
    }
}

REGISTER_LOOP_FUNCTIONS(CMPGAEmergentBehaviorLoopFunctions, "mpga_emergent_behavior_loop_functions")
