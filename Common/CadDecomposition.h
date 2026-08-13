#pragma once

#include "CadNode.h"

#include <QJsonObject>
#include <QString>

#include <string>

void generateVHACDStub(const QString& nodeName,
                       const QJsonObject& params,
                       CadNode* node);
void generateCoACDStub(const QString& nodeName,
                       double concavity,
                       double alpha,
                       double beta,
                       CadNode* node,
                       int maxConvexHull,
                       std::string preprocess,
                       int prepRes,
                       int sampleRes,
                       int mctsNodes,
                       int mctsIter,
                       int mctsDepth,
                       bool pca,
                       bool merge,
                       bool decimate,
                       int maxChVertex,
                       bool extrude,
                       double extrudeMargin,
                       std::string apxMode,
                       int seed);
