//
// Created by djcupo on 4/18/19.
//

#include "analysis.h"
#include <cmath>
#include <numeric>
#include <vector>

CAnalysis::AnalysisResults CAnalysis::AnalyzeAll() {
    AnalysisResults results;

    results.Distance = AnalyzeDistance();
    results.Sparseness = AnalyzeSparseness();

    return results;
}

float CAnalysis::AnalyzeSparseness() {
    //Look I know there are chances that they might be different sizes, but if that happens something else is fucked
    int size = (int) m_vecRobotX.size();

    double centroidX = 0.0;
    double centroidY = 0.0;

    for (int i = 0; i < size; ++i) {
        centroidX += m_vecRobotX[i];
        centroidY += m_vecRobotY[i];
    }

    centroidX /= (double) size;
    centroidY /= (double) size;

    double averageSparseness = 0.0;
    for (int i = 0; i < size; ++i) {
        double distance = sqrt(pow(m_vecRobotX[i] - centroidX, 2) + pow(m_vecRobotY[i] - centroidY, 2));
        averageSparseness += distance;
    }

    averageSparseness /= (double) size;
    return (float) averageSparseness;
}

float CAnalysis::AnalyzeDistance() {
    //See above
    int size = (int) m_vecRobotX.size();



    std::vector<float> vecDistances;

    for (int startPoint = 0; startPoint < size; ++startPoint) {
        for (int referencePoint = 0; referencePoint < size; ++referencePoint) {
            float distance = sqrt(pow(m_vecRobotX[startPoint] - m_vecRobotX[referencePoint], 2) + pow(m_vecRobotY[startPoint] - m_vecRobotY[referencePoint], 2));
            vecDistances.push_back(distance);
        }

    }

    float avgDistance = accumulate(vecDistances.begin(), vecDistances.end(), 0.0) / vecDistances.size();

    return avgDistance;
}
