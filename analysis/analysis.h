//
// Created by djcupo on 4/18/19.
//

#ifndef EMERGENT_BEHAVIOR_CANALYSIS_H
#define EMERGENT_BEHAVIOR_CANALYSIS_H

#include <map>

class CAnalysis {
public:
    CAnalysis(const std::map<int, float> &vecRobotX,
              const std::map<int, float> &vecRobotY,
              const std::map<int, float> &vecRobotZ) :
            m_vecRobotX(vecRobotX),
            m_vecRobotY(vecRobotY),
            m_vecRobotZ(vecRobotZ) {}

    virtual ~CAnalysis() {}

    typedef struct {
        float Sparseness;
        float Distance;
    } AnalysisResults;

    AnalysisResults AnalyzeAll();

private:
    std::map<int, float> m_vecRobotX;
    std::map<int, float> m_vecRobotY;
    std::map<int, float> m_vecRobotZ;

    float AnalyzeSparseness();
    float AnalyzeDistance();


};


#endif //EMERGENT_BEHAVIOR_CANALYSIS_H
