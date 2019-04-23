//
// Created by djcupo on 4/18/19.
//

#ifndef EMERGENT_BEHAVIOR_CANALYSIS_H
#define EMERGENT_BEHAVIOR_CANALYSIS_H

#include <map>
#include <vector>

class CAnalysis {
public:
    CAnalysis(const std::vector<float> &vecRobotX, const std::vector<float> &vecRobotY,
                  const std::vector<float> &vecRobotZ, const std::vector<float> &vecRobotSpeed,
                  const std::vector<int> &vecRobotState) :
            m_vecRobotX(vecRobotX),
            m_vecRobotY(vecRobotY),
            m_vecRobotZ(vecRobotZ),
            m_vecRobotSpeed(vecRobotSpeed),
            m_vecRobotState(vecRobotState){}

    virtual ~CAnalysis() {}

    typedef struct {
        int size = 6;
        float Sparseness;
        float Distance;
        float RadialStdDev;
        float Speed;
        float AngularMomentum;
        int StateZeroCount;
    } AnalysisResults;

    AnalysisResults AnalyzeAll();

private:
    std::vector<float> m_vecRobotX;
    std::vector<float> m_vecRobotY;
    std::vector<float> m_vecRobotZ;
    std::vector<float> m_vecRobotSpeed;
    std::vector<int> m_vecRobotState;

    float sparseness;
    float distance;
    float radialStdDev;
    float speed;
    float angMomentum;
    int state0Count;

    double centroidX;
    double centroidY;

    void AnalyzeSparseness();
    void AnalyzeDistance();
    void AnalyzeSpeed();
    void AnalyzeAngMomentum();
    void AnalyzeState();

};


#endif //EMERGENT_BEHAVIOR_CANALYSIS_H
