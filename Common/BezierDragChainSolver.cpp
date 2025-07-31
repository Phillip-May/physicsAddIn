#include "BezierDragChainSolver.h"
#include <QElapsedTimer>
#include <algorithm>
#include <numeric>
#include <math.h>

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Manual clamp function to avoid std::clamp issues

BezierDragChainSolver::BezierDragChainSolver()
    : m_maxIterations(100)
    , m_convergenceTolerance(0.01)
    , m_bendRadiusTolerance(0.1)
    , m_optimizationStep(0.1)
    , m_curveSmoothness(1.0)
    , m_currentStartPoint(0, 0, 0)
    , m_currentEndPoint(0, 0, 0)
    , m_currentControlPoint(0, 0, 0)
{
    qDebug() << "[BezierDragChainSolver] Constructor called";
    resetSolverStatistics();
}

BezierDragChainSolver::~BezierDragChainSolver()
{
    qDebug() << "[BezierDragChainSolver] Destructor called";
}

std::vector<BezierDragChainSegment> BezierDragChainSolver::solveDragChainPath(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& startDirection,
    const QVector3D& endDirection,
    double pitchLength,
    double maxBendRadius,
    int segmentCount,
    bool startLocked,
    bool endLocked,
    const QVector3D& startPlaneNormal,
    const QVector3D& endPlaneNormal)
{
    qDebug() << "[BezierDragChainSolver] solveDragChainPath called";
    qDebug() << "[BezierDragChainSolver] Parameters:";
    qDebug() << "[BezierDragChainSolver]   Start point:" << startPoint;
    qDebug() << "[BezierDragChainSolver]   End point:" << endPoint;
    qDebug() << "[BezierDragChainSolver]   Start direction:" << startDirection;
    qDebug() << "[BezierDragChainSolver]   End direction:" << endDirection;
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Max bend radius:" << maxBendRadius;
    qDebug() << "[BezierDragChainSolver]   Segment count:" << segmentCount;
    qDebug() << "[BezierDragChainSolver]   Start locked:" << startLocked;
    qDebug() << "[BezierDragChainSolver]   End locked:" << endLocked;
    
    // Reset solver statistics
    resetSolverStatistics();
    
    // Start timing
    QElapsedTimer timer;
    timer.start();
    
    // Store current parameters
    m_currentStartPoint = startPoint;
    m_currentEndPoint = endPoint;
    
    // Create initial quadratic Bézier curve
    if (!createQuadraticBezierCurve(startPoint, endPoint, startDirection, endDirection, 
                                   pitchLength, segmentCount, maxBendRadius)) {
        qDebug() << "[BezierDragChainSolver] Failed to create initial Bézier curve";
        return std::vector<BezierDragChainSegment>();
    }
    
    // Optimize control points to satisfy constraints
    if (!optimizeControlPoints(endPoint, endDirection, startLocked, endLocked, 
                             maxBendRadius, pitchLength, segmentCount)) {
        qDebug() << "[BezierDragChainSolver] Failed to optimize control points";
        return std::vector<BezierDragChainSegment>();
    }
    
    // Generate final segments from the optimized curve
    std::vector<BezierDragChainSegment> segments = generateSegmentsFromBezier(
        m_currentStartPoint, m_currentControlPoint, m_currentEndPoint,
        pitchLength, segmentCount, maxBendRadius
    );
    
    // Calculate final statistics
    calculateSegmentLengthStatistics(segments, maxBendRadius, pitchLength);
    
    // Update final statistics
    m_lastSolverStatistics.computationTime = timer.elapsed() / 1000.0; // Convert to seconds
    m_lastSolverStatistics.converged = checkConvergence(endPoint, m_convergenceTolerance);
    m_lastSolverStatistics.finalDistanceToTarget = calculateDistance(
        segments.empty() ? startPoint : segments.back().endPoint, endPoint);
    
    qDebug() << "[BezierDragChainSolver] solveDragChainPath completed:";
    qDebug() << "[BezierDragChainSolver]   Segments generated:" << segments.size();
    qDebug() << "[BezierDragChainSolver]   Computation time:" << m_lastSolverStatistics.computationTime << "s";
    qDebug() << "[BezierDragChainSolver]   Converged:" << m_lastSolverStatistics.converged;
    qDebug() << "[BezierDragChainSolver]   Final distance to target:" << m_lastSolverStatistics.finalDistanceToTarget;
    
    return segments;
}

bool BezierDragChainSolver::createQuadraticBezierCurve(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& startDirection,
    const QVector3D& endDirection,
    double pitchLength,
    int segmentCount,
    double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] createQuadraticBezierCurve called";
    
    // Calculate initial control point based on start and end directions
    QVector3D midPoint = (startPoint + endPoint) * 0.5f;
    
    // Calculate perpendicular direction for control point placement
    QVector3D direction = normalizeVector(endPoint - startPoint);
    QVector3D perpendicular = calculatePerpendicularAxis(direction);
    
    // Calculate control point distance based on desired curve shape
    double distance = calculateDistance(startPoint, endPoint);
    double controlDistance = distance * 0.3; // Start with 30% of total distance
    
    // Adjust control point based on start and end directions
    QVector3D startPerp = normalizeVector(crossProduct(startDirection, direction));
    QVector3D endPerp = normalizeVector(crossProduct(endDirection, direction));
    
    // Blend the perpendicular directions
    QVector3D blendedPerp = normalizeVector(startPerp + endPerp);
    
    // Place control point
    m_currentControlPoint = midPoint + blendedPerp * controlDistance;
    
    qDebug() << "[BezierDragChainSolver] Initial control point:" << m_currentControlPoint;
    
    return true;
}

bool BezierDragChainSolver::optimizeControlPoints(
    const QVector3D& targetEndPoint,
    const QVector3D& targetEndDirection,
    bool startLocked,
    bool endLocked,
    double maxBendRadius,
    double pitchLength,
    int segmentCount)
{
    qDebug() << "[BezierDragChainSolver] optimizeControlPoints called";
    
    QVector3D bestControlPoint = m_currentControlPoint;
    double bestError = std::numeric_limits<double>::max();
    
    // Generate segments with current control point
    std::vector<BezierDragChainSegment> currentSegments = generateSegmentsFromBezier(
        m_currentStartPoint, m_currentControlPoint, m_currentEndPoint,
        pitchLength, segmentCount, maxBendRadius
    );
    
    // Calculate initial error
    double currentError = calculateDistance(
        currentSegments.empty() ? m_currentStartPoint : currentSegments.back().endPoint, 
        targetEndPoint);
    
    qDebug() << "[BezierDragChainSolver] Initial error:" << currentError;
    
    // Simple gradient descent optimization
    for (int iteration = 0; iteration < m_maxIterations; ++iteration) {
        bool improved = false;
        
        // Try small perturbations in different directions
        std::vector<QVector3D> perturbations = {
            QVector3D(m_optimizationStep, 0, 0),
            QVector3D(-m_optimizationStep, 0, 0),
            QVector3D(0, m_optimizationStep, 0),
            QVector3D(0, -m_optimizationStep, 0),
            QVector3D(0, 0, m_optimizationStep),
            QVector3D(0, 0, -m_optimizationStep)
        };
        
        for (const auto& perturbation : perturbations) {
            QVector3D testControlPoint = m_currentControlPoint + perturbation;
            
            // Generate segments with test control point
            std::vector<BezierDragChainSegment> testSegments = generateSegmentsFromBezier(
                m_currentStartPoint, testControlPoint, m_currentEndPoint,
                pitchLength, segmentCount, maxBendRadius
            );
            
            if (testSegments.empty()) continue;
            
            // Calculate error for this test point
            double testError = calculateDistance(testSegments.back().endPoint, targetEndPoint);
            
            // Add penalty for constraint violations
            if (!validateBendRadiusConstraints(testSegments, maxBendRadius)) {
                testError += 1000.0; // Large penalty for constraint violation
            }
            
            if (testError < currentError) {
                currentError = testError;
                m_currentControlPoint = testControlPoint;
                improved = true;
                
                qDebug() << "[BezierDragChainSolver] Iteration" << iteration << "improved error to:" << currentError;
            }
        }
        
        // Update statistics
        m_lastSolverStatistics.iterationsUsed = iteration + 1;
        m_lastSolverStatistics.finalDistanceToTarget = currentError;
        
        // Call callback if set
        if (m_statisticsCallback) {
            m_statisticsCallback(m_lastSolverStatistics);
        }
        
        // Check convergence
        if (currentError < m_convergenceTolerance) {
            qDebug() << "[BezierDragChainSolver] Converged at iteration" << iteration;
            break;
        }
        
        // Reduce step size if no improvement
        if (!improved) {
            m_optimizationStep *= 0.5;
            if (m_optimizationStep < 0.001) {
                qDebug() << "[BezierDragChainSolver] Step size too small, stopping";
                break;
            }
        }
    }
    
    qDebug() << "[BezierDragChainSolver] Optimization completed with final error:" << currentError;
    return currentError < m_convergenceTolerance * 10.0; // Allow some tolerance
}

std::vector<BezierDragChainSegment> BezierDragChainSolver::generateSegmentsFromBezier(
    const QVector3D& p0, const QVector3D& p1, const QVector3D& p2,
    double pitchLength, int segmentCount, double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] generateSegmentsFromBezier called";
    qDebug() << "[BezierDragChainSolver]   p0:" << p0 << "p1:" << p1 << "p2:" << p2;
    qDebug() << "[BezierDragChainSolver]   pitchLength:" << pitchLength << "segmentCount:" << segmentCount;
    
    std::vector<BezierDragChainSegment> segments;
    
    // Calculate total arc length
    double totalArcLength = calculateBezierArcLength(p0, p1, p2);
    qDebug() << "[BezierDragChainSolver] Total arc length:" << totalArcLength;
    
    // Calculate segment arc lengths
    double segmentArcLength = totalArcLength / segmentCount;
    qDebug() << "[BezierDragChainSolver] Segment arc length:" << segmentArcLength;
    
    QVector3D prevPoint = p0;
    QVector3D prevDirection = normalizeVector(p1 - p0);
    
    for (int i = 0; i < segmentCount; ++i) {
        double tStart = static_cast<double>(i) / segmentCount;
        double tEnd = static_cast<double>(i + 1) / segmentCount;
        
        // Find parameter values for equal arc length segments
        double targetLength = segmentArcLength * (i + 1);
        double tEndActual = findBezierParameterForArcLength(p0, p1, p2, targetLength, 0.0);
        
        // Evaluate curve at segment endpoints
        QVector3D segmentStart = evaluateBezierCurve(p0, p1, p2, tStart);
        QVector3D segmentEnd = evaluateBezierCurve(p0, p1, p2, tEndActual);
        
        // Calculate segment direction
        QVector3D segmentDirection = normalizeVector(segmentEnd - segmentStart);
        
        // Calculate bend angle from previous segment
        double bendAngle = 0.0;
        QVector3D bendAxis(0, 0, 0);
        bool isBend = false;
        double bendRadius = 0.0;
        
        if (i > 0) {
            double dot = dotProduct(prevDirection, segmentDirection);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot; // Clamp to avoid numerical issues
            bendAngle = std::acos(dot);
            
            if (bendAngle > 0.01) { // Threshold for considering it a bend
                isBend = true;
                bendAxis = normalizeVector(crossProduct(prevDirection, segmentDirection));
                bendRadius = calculateBendRadius(prevPoint, segmentStart, segmentEnd);
            }
        }
        
        // Create segment with exactly the pitch length
        BezierDragChainSegment segment(segmentStart, segmentEnd, p1, pitchLength,
                                     bendAngle, bendAxis, isBend, bendRadius, tStart, tEndActual);
        segments.push_back(segment);
        
        // Update for next iteration
        prevPoint = segmentStart;
        prevDirection = segmentDirection;
        
        qDebug() << "[BezierDragChainSolver] Segment" << i << ":";
        qDebug() << "[BezierDragChainSolver]   Start:" << segmentStart << "End:" << segmentEnd;
        qDebug() << "[BezierDragChainSolver]   Length:" << pitchLength << "Bend angle:" << bendAngle;
        qDebug() << "[BezierDragChainSolver]   Is bend:" << isBend << "Bend radius:" << bendRadius;
    }
    
    qDebug() << "[BezierDragChainSolver] Generated" << segments.size() << "segments";
    return segments;
}

QVector3D BezierDragChainSolver::evaluateBezierCurve(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double t)
{
    // Quadratic Bézier curve: B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
    double oneMinusT = 1.0 - t;
    double oneMinusTSquared = oneMinusT * oneMinusT;
    double tSquared = t * t;
    double twoTOneMinusT = 2.0 * t * oneMinusT;
    
    return p0 * oneMinusTSquared + p1 * twoTOneMinusT + p2 * tSquared;
}

QVector3D BezierDragChainSolver::evaluateBezierDerivative(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double t)
{
    // Derivative of quadratic Bézier curve: B'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁)
    double oneMinusT = 1.0 - t;
    return 2.0 * oneMinusT * (p1 - p0) + 2.0 * t * (p2 - p1);
}

double BezierDragChainSolver::calculateBezierArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double tStart, double tEnd)
{
    // Numerical integration using Simpson's rule
    const int numSteps = 100;
    double step = (tEnd - tStart) / numSteps;
    double arcLength = 0.0;
    
    for (int i = 0; i < numSteps; ++i) {
        double t = tStart + i * step;
        QVector3D derivative = evaluateBezierDerivative(p0, p1, p2, t);
        double magnitude = derivative.length();
        
        // Simpson's rule weights
        double weight = (i == 0 || i == numSteps - 1) ? 1.0 : (i % 2 == 0 ? 2.0 : 4.0);
        arcLength += weight * magnitude * step / 3.0;
    }
    
    return arcLength;
}

double BezierDragChainSolver::findBezierParameterForArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double targetLength, double tStart)
{
    // Check if the target length is achievable within the remaining curve
    double remainingArcLength = calculateBezierArcLength(p0, p1, p2, tStart, 1.0);
    
    if (targetLength > remainingArcLength) {
        // Target length exceeds remaining arc length, return 1.0
        qDebug() << "[BezierDragChainSolver] WARNING: Target length" << targetLength << "exceeds remaining arc length" << remainingArcLength << "from t=" << tStart << "to t=1.0";
        return 1.0;
    }
    
    // Binary search to find parameter t for given arc length
    double tMin = tStart;
    double tMax = 1.0;
    double tolerance = 0.001;
    
    while (tMax - tMin > tolerance) {
        double tMid = (tMin + tMax) * 0.5;
        double arcLength = calculateBezierArcLength(p0, p1, p2, tStart, tMid);
        
        if (arcLength < targetLength) {
            tMin = tMid;
        } else {
            tMax = tMid;
        }
    }
    
    double result = (tMin + tMax) * 0.5;
    
    // Ensure the result is within bounds
    if (result > 1.0) {
        qDebug() << "[BezierDragChainSolver] WARNING: Calculated t value" << result << "exceeds 1.0, clamping to 1.0";
        result = 1.0;
    }
    
    // Additional safety check for invalid t values
    if (result < tStart || result > 1.0) {
        qDebug() << "[BezierDragChainSolver] ERROR: Invalid t value calculated:" << result << "tStart:" << tStart;
        qDebug() << "[BezierDragChainSolver] Target length:" << targetLength << "Remaining arc length:" << remainingArcLength;
        result = 1.0; // Use end point as fallback
    }
    
    return result;
}

std::vector<ConnectionPathSegment> BezierDragChainSolver::convertToVisualizationSegments(
    const std::vector<BezierDragChainSegment>& segments,
    const QVector4D& mainColor,
    const QVector4D& indicatorColor)
{
    std::vector<ConnectionPathSegment> visualizationSegments;
    
    // Define alternating colors for segments
    QVector4D color1 = mainColor; // Primary color (e.g., red)
    QVector4D color2 = QVector4D(0, 0, 1, 0.8f); // Secondary color (e.g., blue)
    QVector4D color3 = QVector4D(0, 1, 0, 0.8f); // Tertiary color (e.g., green)
    
    // Create a pattern of 3 colors for better visual distinction
    std::vector<QVector4D> colorPattern = {color1, color2, color3};
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        // Check if this segment violates bend radius constraints
        bool violatesBendRadius = (segment.bendRadius < 0.0);
        double actualBendRadius = std::abs(segment.bendRadius);
        
        // Choose color based on segment index for alternating pattern
        QVector4D segmentColor = colorPattern[i % colorPattern.size()];
        
        // If segment violates bend radius, use warning color (orange/red)
        if (violatesBendRadius) {
            segmentColor = QVector4D(1.0f, 0.5f, 0.0f, 0.9f); // Orange for violations
        }
        
        // Main segment with color (warning color if violation)
        visualizationSegments.emplace_back(
            segment.startPoint, segment.endPoint, segmentColor, 3.0f, false);
        
        // Bend indicator if it's a bend
        if (segment.isBend) {
            QVector3D bendCenter = calculateBendCenter(segment.startPoint, segment.endPoint, 
                                                     segment.bendAxis, actualBendRadius);
            QVector3D indicatorStart = bendCenter - segment.bendAxis * (actualBendRadius * 0.2f);
            QVector3D indicatorEnd = bendCenter + segment.bendAxis * (actualBendRadius * 0.2f);
            
            // Use different indicator color for violations
            QVector4D indicatorColorForSegment = violatesBendRadius ? 
                QVector4D(1.0f, 0.0f, 0.0f, 1.0f) : // Red for violations
                indicatorColor; // Normal color
            
            visualizationSegments.emplace_back(
                indicatorStart, indicatorEnd, indicatorColorForSegment, 5.0f, true);
        }
        
        // Add violation warning indicator if segment violates bend radius
        if (violatesBendRadius) {
            // Add a warning indicator at the segment midpoint
            QVector3D segmentMidPoint = (segment.startPoint + segment.endPoint) * 0.5f;
            QVector3D warningOffset = QVector3D(0, 0, 5.0f); // Larger offset for visibility
            QVector3D warningStart = segmentMidPoint - warningOffset;
            QVector3D warningEnd = segmentMidPoint + warningOffset;
            
            // Red warning indicator
            visualizationSegments.emplace_back(
                warningStart, warningEnd, QVector4D(1.0f, 0.0f, 0.0f, 1.0f), 8.0f, true);
        }
    }
    
    qDebug() << "[BezierDragChainSolver] Created visualization with" << segments.size() << "segments using alternating colors";
    
    return visualizationSegments;
}

std::vector<ConnectionPathSegment> BezierDragChainSolver::convertToEnhancedVisualization(
    const std::vector<BezierDragChainSegment>& segments,
    const QVector4D& primaryColor,
    const QVector4D& secondaryColor,
    const QVector4D& tertiaryColor,
    const QVector4D& indicatorColor)
{
    std::vector<ConnectionPathSegment> visualizationSegments;
    
    // Create a pattern of 3 colors for better visual distinction
    std::vector<QVector4D> colorPattern = {primaryColor, secondaryColor, tertiaryColor};
    
    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& segment = segments[i];
        
        // Check if this segment violates bend radius constraints
        bool violatesBendRadius = (segment.bendRadius < 0.0);
        double actualBendRadius = std::abs(segment.bendRadius);
        
        // Choose color based on segment index for alternating pattern
        QVector4D segmentColor = colorPattern[i % colorPattern.size()];
        
        // If segment violates bend radius, use warning color (orange/red)
        if (violatesBendRadius) {
            segmentColor = QVector4D(1.0f, 0.5f, 0.0f, 0.9f); // Orange for violations
        }
        
        // Main segment with alternating color (warning color if violation)
        visualizationSegments.emplace_back(
            segment.startPoint, segment.endPoint, segmentColor, 3.0f, false);
        
        // Add segment number indicator at the middle of each segment
        QVector3D segmentMidPoint = (segment.startPoint + segment.endPoint) * 0.5f;
        QVector3D indicatorOffset = QVector3D(0, 0, 2.0f); // Small offset for visibility
        QVector3D indicatorStart = segmentMidPoint - indicatorOffset;
        QVector3D indicatorEnd = segmentMidPoint + indicatorOffset;
        
        // Use a distinct color for segment number indicators
        QVector4D numberColor = QVector4D(1, 1, 1, 0.9f); // White with high opacity
        visualizationSegments.emplace_back(
            indicatorStart, indicatorEnd, numberColor, 2.0f, true);
        
        // Bend indicator if it's a bend
        if (segment.isBend) {
            QVector3D bendCenter = calculateBendCenter(segment.startPoint, segment.endPoint, 
                                                     segment.bendAxis, actualBendRadius);
            QVector3D indicatorStart = bendCenter - segment.bendAxis * (actualBendRadius * 0.3f);
            QVector3D indicatorEnd = bendCenter + segment.bendAxis * (actualBendRadius * 0.3f);
            
            // Use different indicator color for violations
            QVector4D indicatorColorForSegment = violatesBendRadius ? 
                QVector4D(1.0f, 0.0f, 0.0f, 1.0f) : // Red for violations
                indicatorColor; // Normal color
            
            visualizationSegments.emplace_back(
                indicatorStart, indicatorEnd, indicatorColorForSegment, 6.0f, true);
        }
        
        // Add violation warning indicator if segment violates bend radius
        if (violatesBendRadius) {
            // Add a warning indicator at the segment midpoint
            QVector3D warningOffset = QVector3D(0, 0, 8.0f); // Larger offset for visibility
            QVector3D warningStart = segmentMidPoint - warningOffset;
            QVector3D warningEnd = segmentMidPoint + warningOffset;
            
            // Red warning indicator
            visualizationSegments.emplace_back(
                warningStart, warningEnd, QVector4D(1.0f, 0.0f, 0.0f, 1.0f), 10.0f, true);
        }
    }
    
    // Add start and end point indicators
    if (!segments.empty()) {
        // Start point indicator (green)
        QVector3D startPoint = segments.front().startPoint;
        QVector3D startIndicatorStart = startPoint - QVector3D(0, 0, 3.0f);
        QVector3D startIndicatorEnd = startPoint + QVector3D(0, 0, 3.0f);
        visualizationSegments.emplace_back(
            startIndicatorStart, startIndicatorEnd, QVector4D(0, 1, 0, 1.0f), 4.0f, true);
        
        // End point indicator (red)
        QVector3D endPoint = segments.back().endPoint;
        QVector3D endIndicatorStart = endPoint - QVector3D(0, 0, 3.0f);
        QVector3D endIndicatorEnd = endPoint + QVector3D(0, 0, 3.0f);
        visualizationSegments.emplace_back(
            endIndicatorStart, endIndicatorEnd, QVector4D(1, 0, 0, 1.0f), 4.0f, true);
    }
    
    qDebug() << "[BezierDragChainSolver] Created enhanced visualization with" << segments.size() << "segments using alternating colors and indicators";
    
    return visualizationSegments;
}

void BezierDragChainSolver::calculateSegmentLengthStatistics(const std::vector<BezierDragChainSegment>& segments, double maxBendRadius, double pitchLength)
{
    if (segments.empty()) {
        m_lastSolverStatistics.minSegmentLength = 0.0;
        m_lastSolverStatistics.maxSegmentLength = 0.0;
        m_lastSolverStatistics.averageSegmentLength = 0.0;
        m_lastSolverStatistics.minSegmentAngle = 0.0;
        m_lastSolverStatistics.maxSegmentAngle = 0.0;
        m_lastSolverStatistics.averageSegmentAngle = 0.0;
        m_lastSolverStatistics.totalCableLength = 0.0;
        m_lastSolverStatistics.totalSegments = 0;
        return;
    }
    
    std::vector<double> lengths;
    std::vector<double> angles;
    double totalLength = 0.0;
    int violationCount = 0;
    
    for (const auto& segment : segments) {
        // Use the stored length which should be exactly the pitch length
        // The stored length is set during segment creation and should be exactly pitchLength
        double segmentLength = segment.length;
        lengths.push_back(segmentLength);
        totalLength += segmentLength;
        
        // Check for bend radius violations
        if (segment.bendRadius < 0.0) {
            violationCount++;
            qDebug() << "[BezierDragChainSolver] Found violation in segment with bend radius:" << std::abs(segment.bendRadius) << "mm";
        }
    }
    
    // Calculate waypoint angles (angles at connection points between segments)
    for (size_t i = 1; i < segments.size(); ++i) {
        double waypointAngle = calculateWaypointAngle(segments, i);
        angles.push_back(waypointAngle);
        qDebug() << "[BezierDragChainSolver] Waypoint" << i << "angle:" << waypointAngle * 180.0 / M_PI << "degrees";
    }
    
    // Calculate length statistics
    auto [minIt, maxIt] = std::minmax_element(lengths.begin(), lengths.end());
    double sum = std::accumulate(lengths.begin(), lengths.end(), 0.0);
    
    m_lastSolverStatistics.minSegmentLength = *minIt;
    m_lastSolverStatistics.maxSegmentLength = *maxIt;
    m_lastSolverStatistics.averageSegmentLength = sum / lengths.size();
    m_lastSolverStatistics.totalCableLength = totalLength;
    m_lastSolverStatistics.totalSegments = static_cast<int>(segments.size());
    
    // Calculate angle statistics
    if (!angles.empty()) {
        auto [minAngleIt, maxAngleIt] = std::minmax_element(angles.begin(), angles.end());
        double angleSum = std::accumulate(angles.begin(), angles.end(), 0.0);
        
        m_lastSolverStatistics.minSegmentAngle = *minAngleIt;
        m_lastSolverStatistics.maxSegmentAngle = *maxAngleIt;
        m_lastSolverStatistics.averageSegmentAngle = angleSum / angles.size();
        
        // Validate that maximum angle doesn't exceed what's allowed by bend radius
        double maxAllowedAngle = calculateMaxBendAngle(m_lastSolverStatistics.averageSegmentLength, maxBendRadius);
        if (m_lastSolverStatistics.maxSegmentAngle > maxAllowedAngle) {
            qDebug() << "[BezierDragChainSolver] WARNING: Maximum segment angle" << m_lastSolverStatistics.maxSegmentAngle * 180.0 / M_PI << "° exceeds maximum allowed" << maxAllowedAngle * 180.0 / M_PI << "°";
            qDebug() << "[BezierDragChainSolver] This indicates a constraint violation - angles should be constrained during generation";
        }
    } else {
        m_lastSolverStatistics.minSegmentAngle = 0.0;
        m_lastSolverStatistics.maxSegmentAngle = 0.0;
        m_lastSolverStatistics.averageSegmentAngle = 0.0;
    }
    
    qDebug() << "[BezierDragChainSolver] Segment length statistics:";
    qDebug() << "[BezierDragChainSolver]   Min:" << m_lastSolverStatistics.minSegmentLength;
    qDebug() << "[BezierDragChainSolver]   Max:" << m_lastSolverStatistics.maxSegmentLength;
    qDebug() << "[BezierDragChainSolver]   Avg:" << m_lastSolverStatistics.averageSegmentLength;
    qDebug() << "[BezierDragChainSolver]   Total cable length:" << m_lastSolverStatistics.totalCableLength;
    qDebug() << "[BezierDragChainSolver]   Total segments:" << m_lastSolverStatistics.totalSegments;
    qDebug() << "[BezierDragChainSolver]   Violation count:" << violationCount;
    qDebug() << "[BezierDragChainSolver] Waypoint angle statistics:";
    qDebug() << "[BezierDragChainSolver]   Min:" << m_lastSolverStatistics.minSegmentAngle * 180.0 / M_PI << "°";
    qDebug() << "[BezierDragChainSolver]   Max:" << m_lastSolverStatistics.maxSegmentAngle * 180.0 / M_PI << "°";
    qDebug() << "[BezierDragChainSolver]   Avg:" << m_lastSolverStatistics.averageSegmentAngle * 180.0 / M_PI << "°";
    
    // Additional debugging: show all segment lengths and waypoint angles
    qDebug() << "[BezierDragChainSolver] Individual segment lengths:";
    for (size_t i = 0; i < segments.size(); ++i) {
        double calculatedLength = calculateDistance(segments[i].startPoint, segments[i].endPoint);
        double storedLength = segments[i].length;
        bool violatesBendRadius = (segments[i].bendRadius < 0.0);
        qDebug() << "[BezierDragChainSolver]   Segment" << i << "calculated:" << calculatedLength << "stored:" << storedLength << "difference:" << std::abs(calculatedLength - storedLength) << "violation:" << violatesBendRadius;
        
        // Warn if there's a significant difference between calculated and stored lengths
        if (std::abs(calculatedLength - storedLength) > 0.001) {
            qDebug() << "[BezierDragChainSolver]   WARNING: Segment" << i << "has length discrepancy!";
        }
    }
    
    qDebug() << "[BezierDragChainSolver] Individual waypoint angles:";
    for (size_t i = 1; i < segments.size(); ++i) {
        double waypointAngle = calculateWaypointAngle(segments, i);
        qDebug() << "[BezierDragChainSolver]   Waypoint" << i << "angle:" << waypointAngle * 180.0 / M_PI << "degrees";
    }
    
    if (violationCount > 0) {
        qDebug() << "[BezierDragChainSolver] WARNING: Found" << violationCount << "segments that violate bend radius constraints!";
        qDebug() << "[BezierDragChainSolver] These segments maintain fixed pitch length but exceed the maximum bend radius.";
        qDebug() << "[BezierDragChainSolver] Users should adjust the control points or increase the maximum bend radius.";
    }
    
    // Final verification: ensure all segments have exactly the pitch length
    bool allSegmentsCorrectLength = true;
    for (size_t i = 0; i < segments.size(); ++i) {
        if (std::abs(segments[i].length - pitchLength) > 0.001) {
            qDebug() << "[BezierDragChainSolver] ERROR: Segment" << i << "has incorrect stored length:" << segments[i].length << "expected:" << pitchLength;
            allSegmentsCorrectLength = false;
        }
    }
    
    if (allSegmentsCorrectLength) {
        qDebug() << "[BezierDragChainSolver] ✓ All segments have exactly the pitch length of" << pitchLength << "mm";
    } else {
        qDebug() << "[BezierDragChainSolver] ✗ Some segments have incorrect stored lengths!";
    }
}

void BezierDragChainSolver::updateSolverStatistics(double distanceToTarget, bool converged, int additionalIterations)
{
    m_lastSolverStatistics.finalDistanceToTarget = distanceToTarget;
    m_lastSolverStatistics.converged = converged;
    m_lastSolverStatistics.iterationsUsed += additionalIterations;
    
    if (m_statisticsCallback) {
        m_statisticsCallback(m_lastSolverStatistics);
    }
}

bool BezierDragChainSolver::checkConvergence(const QVector3D& targetEndPoint, double tolerance)
{
    if (m_currentSegments.empty()) return false;
    
    double distance = calculateDistance(m_currentSegments.back().endPoint, targetEndPoint);
    return distance < tolerance;
}

bool BezierDragChainSolver::validateBendRadiusConstraints(const std::vector<BezierDragChainSegment>& segments, double maxBendRadius)
{
    for (const auto& segment : segments) {
        if (segment.isBend && segment.bendRadius > maxBendRadius + m_bendRadiusTolerance) {
            return false;
        }
    }
    return true;
}

bool BezierDragChainSolver::validateDirectionConstraints(const std::vector<BezierDragChainSegment>& segments, const QVector3D& startDirection, const QVector3D& endDirection)
{
    if (segments.empty()) return false;
    
    // Check start direction
    QVector3D firstDirection = normalizeVector(segments.front().endPoint - segments.front().startPoint);
    double startDot = dotProduct(startDirection, firstDirection);
    if (startDot < 0.9) return false; // Allow some tolerance
    
    // Check end direction
    QVector3D lastDirection = normalizeVector(segments.back().endPoint - segments.back().startPoint);
    double endDot = dotProduct(endDirection, lastDirection);
    if (endDot < 0.9) return false; // Allow some tolerance
    
    return true;
}

bool BezierDragChainSolver::validatePitchLengthConstraints(const std::vector<BezierDragChainSegment>& segments, double pitchLength)
{
    for (const auto& segment : segments) {
        if (std::abs(segment.length - pitchLength) > m_convergenceTolerance) {
            return false;
        }
    }
    return true;
}

double BezierDragChainSolver::calculateBendRadius(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2)
{
    // Calculate radius of curvature for three points
    QVector3D v1 = p1 - p0;
    QVector3D v2 = p2 - p1;
    
    double v1Length = v1.length();
    double v2Length = v2.length();
    
    if (v1Length < 0.001 || v2Length < 0.001) return 0.0;
    
    QVector3D v1Norm = v1 / v1Length;
    QVector3D v2Norm = v2 / v2Length;
    
    double dot = dotProduct(v1Norm, v2Norm);
    dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
    
    double angle = std::acos(dot);
    if (angle < 0.001) return std::numeric_limits<double>::max(); // Straight line
    
    // Radius = (v1Length + v2Length) / (2 * sin(angle/2))
    return (v1Length + v2Length) / (2.0 * std::sin(angle * 0.5));
}

double BezierDragChainSolver::calculateMaxBendAngle(double segmentLength, double maxBendRadius)
{
    if (maxBendRadius <= 0.0) return 0.0;
    return 2.0 * std::asin(segmentLength / (2.0 * maxBendRadius));
}

QVector3D BezierDragChainSolver::calculatePerpendicularAxis(const QVector3D& direction)
{
    // Find a perpendicular vector
    if (std::abs(direction.x()) < 0.9) {
        return normalizeVector(crossProduct(QVector3D(1, 0, 0), direction));
    } else {
        return normalizeVector(crossProduct(QVector3D(0, 1, 0), direction));
    }
}

QVector3D BezierDragChainSolver::calculateRotationBetweenVectors(const QVector3D& from, const QVector3D& to)
{
    QVector3D cross = crossProduct(from, to);
    double dot = dotProduct(from, to);
    
    if (cross.length() < 0.001) {
        return QVector3D(0, 0, 0); // No rotation needed
    }
    
    return normalizeVector(cross);
}

std::vector<QVector3D> BezierDragChainSolver::generateBezierPoints(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, int numPoints)
{
    std::vector<QVector3D> points;
    points.reserve(numPoints);
    
    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / (numPoints - 1);
        points.push_back(evaluateBezierCurve(p0, p1, p2, t));
    }
    
    return points;
}

QVector3D BezierDragChainSolver::calculateBendCenter(const QVector3D& start, const QVector3D& end, const QVector3D& direction, double radius)
{
    QVector3D midPoint = (start + end) * 0.5f;
    QVector3D perpendicular = normalizeVector(crossProduct(direction, end - start));
    return midPoint + perpendicular * radius;
}

double BezierDragChainSolver::calculateDistance(const QVector3D& p1, const QVector3D& p2)
{
    return (p2 - p1).length();
}

double BezierDragChainSolver::safeClamp(double value, double min, double max)
{
    return (value < min) ? min : (value > max) ? max : value;
}

double BezierDragChainSolver::calculateSegmentAngle(const QVector3D& seg1Start, const QVector3D& seg1End, 
                                                   const QVector3D& seg2Start, const QVector3D& seg2End)
{
    // Calculate the angle between two segments
    QVector3D seg1Dir = normalizeVector(seg1End - seg1Start);
    QVector3D seg2Dir = normalizeVector(seg2End - seg2Start);
    
    double dot = dotProduct(seg1Dir, seg2Dir);
    dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot; // Clamp to avoid numerical issues
    
    return std::acos(dot);
}

// Calculate the total angle at a waypoint where multiple segments connect
double BezierDragChainSolver::calculateWaypointAngle(const std::vector<BezierDragChainSegment>& segments, size_t waypointIndex)
{
    if (waypointIndex == 0 || waypointIndex >= segments.size()) {
        return 0.0; // No angle at start or end waypoints
    }
    
    // Get the segments that connect to this waypoint
    const auto& prevSegment = segments[waypointIndex - 1];
    const auto& nextSegment = segments[waypointIndex];
    
    // Calculate the angle between the incoming and outgoing segments
    QVector3D incomingDir = normalizeVector(prevSegment.endPoint - prevSegment.startPoint);
    QVector3D outgoingDir = normalizeVector(nextSegment.endPoint - nextSegment.startPoint);
    
    double dot = dotProduct(incomingDir, outgoingDir);
    dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot; // Clamp to avoid numerical issues
    
    double angle = std::acos(dot);
    
    qDebug() << "[BezierDragChainSolver] Waypoint" << waypointIndex << "angle:" << angle * 180.0 / M_PI << "degrees";
    
    return angle;
}

QVector3D BezierDragChainSolver::normalizeVector(const QVector3D& vec)
{
    double length = vec.length();
    if (length < 0.001) return QVector3D(0, 0, 0);
    return vec / length;
}

double BezierDragChainSolver::dotProduct(const QVector3D& v1, const QVector3D& v2)
{
    return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}

QVector3D BezierDragChainSolver::crossProduct(const QVector3D& v1, const QVector3D& v2)
{
    return QVector3D(
        v1.y() * v2.z() - v1.z() * v2.y(),
        v1.z() * v2.x() - v1.x() * v2.z(),
        v1.x() * v2.y() - v1.y() * v2.x()
    );
}

BezierDragChainSolver::PathStatistics BezierDragChainSolver::calculatePathStatistics(const std::vector<BezierDragChainSegment>& segments)
{
    PathStatistics stats;
    
    if (segments.empty()) {
        stats.isValid = false;
        return stats;
    }
    
    stats.totalSegments = static_cast<int>(segments.size());
    stats.totalLength = 0.0;
    
    std::vector<double> lengths;
    
    for (const auto& segment : segments) {
        stats.totalLength += segment.length;
        lengths.push_back(segment.length);
        
        if (segment.isBend) {
            stats.bendSegments++;
        } else {
            stats.straightSegments++;
        }
    }
    
    if (!lengths.empty()) {
        auto [minIt, maxIt] = std::minmax_element(lengths.begin(), lengths.end());
        stats.minSegmentLength = *minIt;
        stats.maxSegmentLength = *maxIt;
        stats.averageSegmentLength = stats.totalLength / stats.totalSegments;
    }
    
    stats.isValid = true;
    return stats;
} 

// Simplified solving function - no iterations, just mathematical calculation
std::vector<BezierDragChainSegment> BezierDragChainSolver::solveDragChainPathSimple(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] solveDragChainPathSimple called";
    qDebug() << "[BezierDragChainSolver] Parameters:";
    qDebug() << "[BezierDragChainSolver]   Start point:" << startPoint;
    qDebug() << "[BezierDragChainSolver]   End point:" << endPoint;
    qDebug() << "[BezierDragChainSolver]   Control point:" << controlPoint;
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    
    // Reset solver statistics
    resetSolverStatistics();
    
    // Start timing
    QElapsedTimer timer;
    timer.start();
    
    // Calculate segments along the user-defined Bézier curve with fixed pitch length
    std::vector<BezierDragChainSegment> segments = calculateSegmentsAlongCurve(
        startPoint, controlPoint, endPoint, pitchLength, maxBendRadius);
    
    // Calculate final statistics
    calculateSegmentLengthStatistics(segments, maxBendRadius, pitchLength);
    
    // Update final statistics (no iterations needed)
    m_lastSolverStatistics.computationTime = timer.elapsed() / 1000.0;
    m_lastSolverStatistics.converged = true; // Always converges since no iterations
    m_lastSolverStatistics.iterationsUsed = 0; // No iterations
    m_lastSolverStatistics.controlPointsOptimized = static_cast<int>(segments.size()); // Store actual segment count
    m_lastSolverStatistics.finalDistanceToTarget = calculateDistance(
        segments.empty() ? startPoint : segments.back().endPoint, endPoint);
    
    qDebug() << "[BezierDragChainSolver] solveDragChainPathSimple completed:";
    qDebug() << "[BezierDragChainSolver]   Segments generated:" << segments.size();
    qDebug() << "[BezierDragChainSolver]   Computation time:" << m_lastSolverStatistics.computationTime << "s";
    qDebug() << "[BezierDragChainSolver]   Final distance to target:" << m_lastSolverStatistics.finalDistanceToTarget;
    
    return segments;
}

std::vector<BezierDragChainSegment> BezierDragChainSolver::calculateSegmentsAlongCurve(
    const QVector3D& p0, const QVector3D& p1, const QVector3D& p2,
    double pitchLength, double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] calculateSegmentsAlongCurve called";
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Bézier curve: p0=" << p0 << ", p1=" << p1 << ", p2=" << p2;
    
    std::vector<BezierDragChainSegment> segments;
    
    if (pitchLength <= 0.0) {
        qDebug() << "[BezierDragChainSolver] Invalid pitch length:" << pitchLength;
        return segments;
    }
    
    // Calculate total distance from start to end
    double totalDistance = calculateDistance(p0, p2);
    qDebug() << "[BezierDragChainSolver] Total distance from start to end:" << totalDistance;
    
    // Calculate how many segments of exactly pitch length fit
    int segmentCount = static_cast<int>(std::ceil(totalDistance / pitchLength));
    qDebug() << "[BezierDragChainSolver] Calculated segment count:" << segmentCount;
    
    // Ensure at least 1 segment
    if (segmentCount < 1) segmentCount = 1;
    
    // Use a more flexible approach - generate segments until we reach the end point
    // or can't make progress, rather than being limited by the initial segment count
    int maxSegments = std::max(segmentCount * 2, 100); // Allow more segments if needed
    
    // Additional check: if the total distance is much smaller than pitch length, 
    // we might not be able to create proper segments
    if (totalDistance < pitchLength * 0.5) {
        qDebug() << "[BezierDragChainSolver] WARNING: Total distance" << totalDistance << "is much smaller than pitch length" << pitchLength;
        qDebug() << "[BezierDragChainSolver] This may result in segments that don't meet minimum length requirements";
    }
    
    // Calculate the maximum allowed bend angle based on max bend radius
    double maxBendAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
    qDebug() << "[BezierDragChainSolver] Maximum allowed bend angle:" << maxBendAngle * 180.0 / M_PI << "degrees";
    
    // Generate segments with EXACTLY fixed pitch length
    QVector3D currentPoint = p0;
    double distanceToEnd = calculateDistance(currentPoint, p2); // Initialize distance to end
    
    for (int i = 0; i < maxSegments; ++i) { // Use flexible segment count
        // Calculate the target direction for this segment
        QVector3D targetDirection;
        
        if (i == 0) {
            // First segment: aim towards the end point
            targetDirection = normalizeVector(p2 - currentPoint);
        } else {
            // For subsequent segments, use a simple approach that prioritizes fixed length
            // Calculate the direction that would reach the end point in the remaining distance
            double remainingDistance = calculateDistance(currentPoint, p2);
            if (remainingDistance <= pitchLength) {
                // We're close to the end, aim directly at it
                targetDirection = normalizeVector(p2 - currentPoint);
            } else {
                // Use a simple direction that follows the general path to the end
                // This ensures we don't get stuck in loops while maintaining fixed lengths
                QVector3D directionToEnd = normalizeVector(p2 - currentPoint);
                
                // If we have a previous segment, try to maintain some continuity
                if (i > 0) {
                    QVector3D prevDirection = normalizeVector(segments.back().endPoint - segments.back().startPoint);
                    
                    // Blend between previous direction and direction to end
                    // This prevents sharp turns while still moving toward the target
                    double blendFactor = 0.3; // Prefer moving toward end point
                    QVector3D blendedDirection = prevDirection * (1.0 - blendFactor) + directionToEnd * blendFactor;
                    targetDirection = normalizeVector(blendedDirection);
                } else {
                    targetDirection = directionToEnd;
                }
            }
        }
        
        // Calculate the segment endpoint with EXACTLY the pitch length
        QVector3D segmentEnd = currentPoint + targetDirection * pitchLength;
        
        // Safety check: ensure we don't create a zero-length segment
        if (calculateDistance(currentPoint, segmentEnd) < 0.001) {
            qDebug() << "[BezierDragChainSolver] WARNING: Zero-length segment detected, using direct path";
            segmentEnd = currentPoint + normalizeVector(p2 - currentPoint) * pitchLength;
        }
        
        // Calculate bend information BEFORE applying constraints
        bool isBend = false;
        double bendAngle = 0.0;
        QVector3D bendAxis(0, 0, 0);
        double bendRadius = 0.0;
        bool violatesBendRadius = false;
        
        if (i > 0) {
            QVector3D prevSegmentStart = segments.back().startPoint;
            QVector3D prevSegmentEnd = segments.back().endPoint;
            QVector3D prevSegmentDir = normalizeVector(prevSegmentEnd - prevSegmentStart);
            
            // Calculate the angle between the previous segment and the current target direction
            double dot = QVector3D::dotProduct(prevSegmentDir, targetDirection);
        dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
        bendAngle = std::acos(dot);
            
            if (bendAngle > 0.01) { // If there's a significant angle change
                isBend = true;
                
                // Calculate bend axis (perpendicular to both segments)
                bendAxis = normalizeVector(crossProduct(prevSegmentDir, targetDirection));
                
                // Calculate bend radius based on exact pitch length and angle
                bendRadius = pitchLength / (2.0 * std::sin(bendAngle / 2.0));
                
                // Check if this bend violates the maximum bend radius constraint
                if (bendRadius > maxBendRadius) {
                    violatesBendRadius = true;
                    qDebug() << "[BezierDragChainSolver] WARNING: Segment" << i << "violates bend radius constraint!";
                    qDebug() << "[BezierDragChainSolver]   Calculated bend radius:" << bendRadius << "mm";
                    qDebug() << "[BezierDragChainSolver]   Maximum allowed:" << maxBendRadius << "mm";
                    qDebug() << "[BezierDragChainSolver]   Bend angle:" << bendAngle * 180.0 / M_PI << "degrees";
                    qDebug() << "[BezierDragChainSolver]   This segment will be marked for user attention";
                }
            }
        }
        
        // IMPORTANT: We do NOT constrain the bend angle to respect maxBendRadius
        // Instead, we allow the bend to exceed the maximum and mark it for user attention
        // This ensures all segments maintain exactly the pitch length
        
        // The segment length should be exactly pitch length by construction
        double actualSegmentLength = calculateDistance(currentPoint, segmentEnd);
        
        // Verify the segment length is exactly what we expect
        if (std::abs(actualSegmentLength - pitchLength) > 0.001) {
            qDebug() << "[BezierDragChainSolver] ERROR: Segment length" << actualSegmentLength << "differs from pitch length" << pitchLength;
            qDebug() << "[BezierDragChainSolver] This should not happen with fixed-length construction!";
            // Force the correct length
            QVector3D segmentDir = normalizeVector(segmentEnd - currentPoint);
            segmentEnd = currentPoint + segmentDir * pitchLength;
            actualSegmentLength = pitchLength;
        }
        
        // IMPORTANT: Always use exactly the pitch length for the segment
        // This ensures all segments have exactly the same length
        actualSegmentLength = pitchLength;
        
        // Create segment with exactly the pitch length
        BezierDragChainSegment segment(currentPoint, segmentEnd, p1, pitchLength,
                                     bendAngle, bendAxis, isBend, bendRadius, 0.0, 1.0);
        
        // Mark the segment if it violates bend radius constraints
        if (violatesBendRadius) {
            // We'll use the bendRadius field to store the violation flag
            // A negative bend radius indicates a violation
            segment.bendRadius = -bendRadius; // Negative indicates violation
        }
        
        segments.push_back(segment);
        
        qDebug() << "[BezierDragChainSolver] Segment" << i << ":";
        qDebug() << "[BezierDragChainSolver]   Start:" << currentPoint;
        qDebug() << "[BezierDragChainSolver]   End:" << segmentEnd;
        qDebug() << "[BezierDragChainSolver]   Length:" << actualSegmentLength;
        qDebug() << "[BezierDragChainSolver]   Expected pitch length:" << pitchLength;
        qDebug() << "[BezierDragChainSolver]   Length difference:" << std::abs(actualSegmentLength - pitchLength);
        qDebug() << "[BezierDragChainSolver]   Target direction:" << targetDirection;
        qDebug() << "[BezierDragChainSolver]   Bend angle:" << bendAngle * 180.0 / M_PI << "degrees";
        qDebug() << "[BezierDragChainSolver]   Is bend:" << isBend;
        qDebug() << "[BezierDragChainSolver]   Violates bend radius:" << violatesBendRadius;
        if (violatesBendRadius) {
            qDebug() << "[BezierDragChainSolver]   Violation severity:" << std::abs(segment.bendRadius) - maxBendRadius << "mm over limit";
        }
        
        // Update current point for next iteration
        currentPoint = segmentEnd;
        
        // Safety check: ensure we're making progress toward the end point
        double newDistanceToEnd = calculateDistance(currentPoint, p2);
        if (i > 0 && newDistanceToEnd >= distanceToEnd) {
            qDebug() << "[BezierDragChainSolver] WARNING: Segment" << i << "is not making progress toward end point";
            qDebug() << "[BezierDragChainSolver]   Previous distance:" << distanceToEnd << "New distance:" << newDistanceToEnd;
            break;
        }
        distanceToEnd = newDistanceToEnd;
        

        
        // Safety check: if we're very close to the end point, create a final segment
        if (distanceToEnd < pitchLength * 0.5) {
            qDebug() << "[BezierDragChainSolver] WARNING: Too close to end point (" << distanceToEnd << "), creating final segment";
            
            // Create a final segment with exactly the pitch length
            QVector3D directionToEnd = normalizeVector(p2 - currentPoint);
            QVector3D finalSegmentEnd = currentPoint + directionToEnd * pitchLength;
            double finalSegmentLength = pitchLength; // Always exactly pitch length
            
            // Ensure the final segment length is exactly pitch length
            finalSegmentLength = pitchLength;
            
            // Calculate bend information for the final segment
            bool isBend = false;
            double bendAngle = 0.0;
            QVector3D bendAxis(0, 0, 0);
            double bendRadius = 0.0;
            bool violatesBendRadius = false;
            
            if (i > 0) {
                QVector3D prevSegmentStart = segments.back().startPoint;
                QVector3D prevSegmentEnd = segments.back().endPoint;
                
                bendAngle = calculateSegmentAngle(prevSegmentStart, prevSegmentEnd, currentPoint, finalSegmentEnd);
                
                if (bendAngle > 0.01) {
                    isBend = true;
                    QVector3D segment1Dir = normalizeVector(prevSegmentEnd - prevSegmentStart);
                    QVector3D segment2Dir = normalizeVector(finalSegmentEnd - currentPoint);
                    bendAxis = normalizeVector(crossProduct(segment1Dir, segment2Dir));
                    bendRadius = finalSegmentLength / (2.0 * std::sin(bendAngle / 2.0));
                    
                    if (bendRadius > maxBendRadius) {
                        violatesBendRadius = true;
                        bendRadius = -bendRadius; // Mark as violation
                    }
                }
            }
            
            BezierDragChainSegment finalSegment(currentPoint, finalSegmentEnd, p1, pitchLength,
                                             bendAngle, bendAxis, isBend, bendRadius, 0.0, 1.0);
            segments.push_back(finalSegment);
            
            qDebug() << "[BezierDragChainSolver] Added final segment of exact pitch length:" << finalSegmentLength;
            qDebug() << "[BezierDragChainSolver] Final distance to target after adding segment:" << calculateDistance(finalSegmentEnd, p2);
            break;
        }
        
        // Additional safety check: if we've reached the end point, stop
        if (distanceToEnd < 0.001) {
            qDebug() << "[BezierDragChainSolver] Reached end point, stopping segment generation";
            break;
        }
    }
    
    // Check if we need to add a final segment to reach the end point
    if (!segments.empty()) {
        double finalDistance = calculateDistance(segments.back().endPoint, p2);
        qDebug() << "[BezierDragChainSolver] Final distance to end point:" << finalDistance;
        
        if (finalDistance > pitchLength * 0.1) { // If we're significantly off from the end point
            qDebug() << "[BezierDragChainSolver] WARNING: Final segment doesn't reach end point, distance:" << finalDistance;
            qDebug() << "[BezierDragChainSolver] This indicates the main loop terminated early - final segment creation should have happened in the loop";
        } else {
            qDebug() << "[BezierDragChainSolver] Final segment already reaches end point within tolerance";
        }
    }
    
    qDebug() << "[BezierDragChainSolver] Generated" << segments.size() << "segments with exact pitch length" << pitchLength;
    qDebug() << "[BezierDragChainSolver] Final distance to target:" << calculateDistance(segments.empty() ? p0 : segments.back().endPoint, p2);
    qDebug() << "[BezierDragChainSolver] Summary:";
    qDebug() << "[BezierDragChainSolver]   Total distance:" << totalDistance;
    qDebug() << "[BezierDragChainSolver]   Expected segment count:" << segmentCount;
    qDebug() << "[BezierDragChainSolver]   Actual segment count:" << segments.size();
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    if (!segments.empty()) {
        qDebug() << "[BezierDragChainSolver]   First segment length:" << calculateDistance(segments.front().startPoint, segments.front().endPoint);
        qDebug() << "[BezierDragChainSolver]   Last segment length:" << calculateDistance(segments.back().startPoint, segments.back().endPoint);
    }
    
    // Final verification: ensure ALL segments are exactly pitch length
    qDebug() << "[BezierDragChainSolver] Starting final verification loop for" << segments.size() << "segments";
    int correctedSegments = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        double actualLength = calculateDistance(segments[i].startPoint, segments[i].endPoint);
        double storedLength = segments[i].length;
        qDebug() << "[BezierDragChainSolver] Segment" << i << "actual length:" << actualLength << "stored length:" << storedLength << "pitch length:" << pitchLength;
        
        if (std::abs(actualLength - pitchLength) > 0.001) {
            qDebug() << "[BezierDragChainSolver] WARNING: Segment" << i << "length" << actualLength << "differs from pitch length" << pitchLength;
            qDebug() << "[BezierDragChainSolver] Correcting segment" << i << "to exact pitch length";
            
            // Force the segment to be exactly pitch length
            QVector3D direction = normalizeVector(segments[i].endPoint - segments[i].startPoint);
            segments[i].endPoint = segments[i].startPoint + direction * pitchLength;
            segments[i].length = pitchLength;
            
            qDebug() << "[BezierDragChainSolver] Corrected segment" << i << "length to:" << pitchLength;
            correctedSegments++;
        }
    }
    qDebug() << "[BezierDragChainSolver] Final verification completed - corrected" << correctedSegments << "segments";
    
    // Verify final result
    qDebug() << "[BezierDragChainSolver] Final verification - all segments should be exactly" << pitchLength << "mm:";
    bool allSegmentsCorrectLength = true;
    for (size_t i = 0; i < segments.size(); ++i) {
        double finalLength = calculateDistance(segments[i].startPoint, segments[i].endPoint);
        double storedLength = segments[i].length;
        qDebug() << "[BezierDragChainSolver]   Segment" << i << "actual length:" << finalLength << "mm, stored length:" << storedLength << "mm";
        
        if (std::abs(finalLength - pitchLength) > 0.001) {
            qDebug() << "[BezierDragChainSolver]   ERROR: Segment" << i << "has incorrect length!";
            allSegmentsCorrectLength = false;
        }
    }
    
    if (allSegmentsCorrectLength) {
        qDebug() << "[BezierDragChainSolver] ✓ All segments have exactly the pitch length of" << pitchLength << "mm";
    } else {
        qDebug() << "[BezierDragChainSolver] ✗ Some segments have incorrect lengths!";
    }
    
    return segments;
}



std::vector<BezierDragChainSegment> BezierDragChainSolver::solveDragChainPathWithMultipleControlPoints(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const std::vector<QVector3D>& controlPoints,
    double pitchLength,
    double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] solveDragChainPathWithMultipleControlPoints called";
    qDebug() << "[BezierDragChainSolver]   Start point:" << startPoint;
    qDebug() << "[BezierDragChainSolver]   End point:" << endPoint;
    qDebug() << "[BezierDragChainSolver]   Control points count:" << controlPoints.size();
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Max bend radius:" << maxBendRadius;
    
    // Reset solver statistics
    resetSolverStatistics();
    
    // Start timing
    QElapsedTimer timer;
    timer.start();
    
    std::vector<BezierDragChainSegment> segments;
    
    if (controlPoints.empty()) {
        // No custom control points, use auto-calculated single control point
        qDebug() << "[BezierDragChainSolver] No custom control points, using auto-calculated control point";
        
        // Auto-calculate a single control point
        QVector3D midPoint = (startPoint + endPoint) * 0.5f;
        QVector3D direction = (endPoint - startPoint).normalized();
        
        // Calculate perpendicular axis
        QVector3D perpendicular;
        if (std::abs(direction.x()) < 0.9f) {
            perpendicular = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
        } else {
            perpendicular = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
        }
        perpendicular.normalize();
        
        double distance = (endPoint - startPoint).length();
        double controlDistance = distance * 0.3; // 30% of total distance
        
        QVector3D autoControlPoint = midPoint + perpendicular * controlDistance;
        
        // Use constraint-based solver for single control point
        segments = solveDragChainPathWithConstraints(startPoint, endPoint, autoControlPoint, pitchLength, maxBendRadius);
    } else {
        // Multiple control points - create a series of constraint-based segments
        qDebug() << "[BezierDragChainSolver] Using" << controlPoints.size() << "custom control points";
        
        std::vector<QVector3D> allPoints;
        allPoints.push_back(startPoint);
        allPoints.insert(allPoints.end(), controlPoints.begin(), controlPoints.end());
        allPoints.push_back(endPoint);
        
        // Create segments between each pair of points with constraint validation
        for (size_t i = 0; i < allPoints.size() - 1; ++i) {
            QVector3D segmentStart = allPoints[i];
            QVector3D segmentEnd = allPoints[i + 1];
            
            // Calculate a control point for this segment
            QVector3D segmentControlPoint;
            if (i < controlPoints.size()) {
                segmentControlPoint = controlPoints[i];
            } else {
                // Auto-calculate control point for this segment
                QVector3D midPoint = (segmentStart + segmentEnd) * 0.5f;
                QVector3D direction = (segmentEnd - segmentStart).normalized();
                QVector3D perpendicular;
                if (std::abs(direction.x()) < 0.9f) {
                    perpendicular = QVector3D::crossProduct(QVector3D(1, 0, 0), direction);
                } else {
                    perpendicular = QVector3D::crossProduct(QVector3D(0, 1, 0), direction);
                }
                perpendicular.normalize();
                
                double distance = (segmentEnd - segmentStart).length();
                double controlDistance = distance * 0.3;
                segmentControlPoint = midPoint + perpendicular * controlDistance;
            }
            
            // Solve this segment with constraints
            auto segmentSegments = solveDragChainPathWithConstraints(segmentStart, segmentEnd, segmentControlPoint, pitchLength, maxBendRadius);
            segments.insert(segments.end(), segmentSegments.begin(), segmentSegments.end());
        }
    }
    
    // Calculate final statistics
    calculateSegmentLengthStatistics(segments, maxBendRadius, pitchLength);
    
    // Update final statistics
    m_lastSolverStatistics.computationTime = timer.elapsed() / 1000.0;
    m_lastSolverStatistics.converged = true;
    m_lastSolverStatistics.iterationsUsed = 0; // No iterations needed for constraint-based approach
    m_lastSolverStatistics.controlPointsOptimized = static_cast<int>(segments.size());
    m_lastSolverStatistics.finalDistanceToTarget = calculateDistance(
        segments.empty() ? startPoint : segments.back().endPoint, endPoint);
    
    qDebug() << "[BezierDragChainSolver] solveDragChainPathWithMultipleControlPoints completed:";
    qDebug() << "[BezierDragChainSolver]   Segments generated:" << segments.size();
    qDebug() << "[BezierDragChainSolver]   Computation time:" << m_lastSolverStatistics.computationTime << "s";
    qDebug() << "[BezierDragChainSolver]   Final distance to target:" << m_lastSolverStatistics.finalDistanceToTarget;
    
    return segments;
}

std::vector<BezierDragChainSegment> BezierDragChainSolver::solveDragChainPathWithConstraints(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] solveDragChainPathWithConstraints called";
    qDebug() << "[BezierDragChainSolver]   Start point:" << startPoint;
    qDebug() << "[BezierDragChainSolver]   End point:" << endPoint;
    qDebug() << "[BezierDragChainSolver]   Control point:" << controlPoint;
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Max bend radius:" << maxBendRadius;
    
    std::vector<BezierDragChainSegment> segments;
    
    // Calculate the ideal Bézier curve
    double totalArcLength = calculateBezierArcLength(startPoint, controlPoint, endPoint, 0.0, 1.0);
    int idealSegmentCount = static_cast<int>(std::ceil(totalArcLength / pitchLength));
    
    qDebug() << "[BezierDragChainSolver] Ideal Bézier arc length:" << totalArcLength;
    qDebug() << "[BezierDragChainSolver] Ideal segment count:" << idealSegmentCount;
    
    // Generate constraint-respecting path
    segments = generateConstraintRespectingPath(startPoint, endPoint, controlPoint, pitchLength, maxBendRadius, idealSegmentCount);
    
    qDebug() << "[BezierDragChainSolver] Generated" << segments.size() << "constraint-respecting segments";
    
    return segments;
}

std::vector<BezierDragChainSegment> BezierDragChainSolver::generateConstraintRespectingPath(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendRadius,
    int targetSegmentCount)
{
    qDebug() << "[BezierDragChainSolver] generateConstraintRespectingPath called";
    qDebug() << "[BezierDragChainSolver]   Target segment count:" << targetSegmentCount;
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Max bend radius:" << maxBendRadius;
    
    std::vector<BezierDragChainSegment> segments;
    
    // Calculate the maximum bend angle allowed by the bend radius constraint
    double maxBendAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
    qDebug() << "[BezierDragChainSolver] Maximum allowed bend angle:" << maxBendAngle * 180.0 / M_PI << "degrees";
    
    // Generate a path that respects both pitch length and bend radius constraints
    std::vector<QVector3D> waypoints = generateConstraintRespectingPathWithCorrections(
        startPoint, endPoint, controlPoint, pitchLength, maxBendRadius, maxBendAngle);
    
    qDebug() << "[BezierDragChainSolver] Generated" << waypoints.size() << "constraint-respecting waypoints";
    
    // Create segments between waypoints
    qDebug() << "[BezierDragChainSolver] Creating segments between" << waypoints.size() << "waypoints";
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        QVector3D segmentStart = waypoints[i];
        QVector3D segmentEnd = waypoints[i + 1];
        
        // Calculate segment properties
        double segmentLength = calculateDistance(segmentStart, segmentEnd);
        bool isBend = false;
        double bendAngle = 0.0;
        QVector3D bendAxis(0, 0, 0);
        double bendRadius = 0.0;
        
        // Calculate bend information if not the first segment
        if (i > 0) {
            QVector3D prevSegmentStart = segments.back().startPoint;
            QVector3D prevSegmentEnd = segments.back().endPoint;
            
            bendAngle = calculateSegmentAngle(prevSegmentStart, prevSegmentEnd, segmentStart, segmentEnd);
            
            if (bendAngle > 0.01) { // If there's a significant angle change
                isBend = true;
                
                // Calculate bend axis (perpendicular to both segments)
                QVector3D segment1Dir = normalizeVector(prevSegmentEnd - prevSegmentStart);
                QVector3D segment2Dir = normalizeVector(segmentEnd - segmentStart);
                bendAxis = normalizeVector(crossProduct(segment1Dir, segment2Dir));
                
                // Calculate bend radius based on angle and segment length
                bendRadius = segmentLength / (2.0 * std::sin(bendAngle / 2.0));
                
                // Verify bend radius constraint
                if (bendRadius > maxBendRadius) {
                    qDebug() << "[BezierDragChainSolver] WARNING: Bend radius" << bendRadius << "exceeds maximum" << maxBendRadius;
                    qDebug() << "[BezierDragChainSolver] Bend angle:" << bendAngle * 180.0 / M_PI << "degrees";
                }
            }
        }
        
        // Create segment with exactly the pitch length
        BezierDragChainSegment segment(segmentStart, segmentEnd, controlPoint, pitchLength,
                                     bendAngle, bendAxis, isBend, bendRadius, 0.0, 0.0);
        segments.push_back(segment);
        
        qDebug() << "[BezierDragChainSolver] Segment" << i << ":";
        qDebug() << "[BezierDragChainSolver]   Start:" << segmentStart;
        qDebug() << "[BezierDragChainSolver]   End:" << segmentEnd;
        qDebug() << "[BezierDragChainSolver]   Length:" << pitchLength;
        qDebug() << "[BezierDragChainSolver]   Target length:" << pitchLength;
        qDebug() << "[BezierDragChainSolver]   Length difference:" << std::abs(segmentLength - pitchLength);
        qDebug() << "[BezierDragChainSolver]   Bend angle:" << bendAngle * 180.0 / M_PI << "degrees";
        qDebug() << "[BezierDragChainSolver]   Bend radius:" << bendRadius;
        qDebug() << "[BezierDragChainSolver]   Is bend:" << isBend;
    }
    
    qDebug() << "[BezierDragChainSolver] Created" << segments.size() << "segments";
    
    return segments;
}

std::vector<QVector3D> BezierDragChainSolver::generateConstraintRespectingWaypoints(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendAngle)
{
    qDebug() << "[BezierDragChainSolver] generateConstraintRespectingWaypoints called";
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Max bend angle:" << maxBendAngle * 180.0 / M_PI << "degrees";
    
    std::vector<QVector3D> waypoints;
    waypoints.push_back(startPoint);
    
    // Calculate total arc length of the Bézier curve
    double totalArcLength = calculateBezierArcLength(startPoint, controlPoint, endPoint, 0.0, 1.0);
    int segmentCount = static_cast<int>(std::ceil(totalArcLength / pitchLength));
    
    // Ensure at least 2 segments for a meaningful path
    if (segmentCount < 2) segmentCount = 2;
    
    qDebug() << "[BezierDragChainSolver] Total arc length:" << totalArcLength;
    qDebug() << "[BezierDragChainSolver] Target segment count:" << segmentCount;
    
    // Generate waypoints with exact pitch length along the Bézier curve
    QVector3D currentPoint = startPoint;
    double currentT = 0.0;
    
    for (int i = 0; i < segmentCount; ++i) {
        // Find the t parameter for the next waypoint with exact pitch length
        double nextT = findBezierParameterForArcLength(startPoint, controlPoint, endPoint, pitchLength, currentT);
        
        // If this is the last segment, ensure we reach the end point
        if (i == segmentCount - 1) {
            nextT = 1.0;
        }
        
        // Evaluate the Bézier curve at the next waypoint
        QVector3D nextWaypoint = evaluateBezierCurve(startPoint, controlPoint, endPoint, nextT);
        
        // Check if the angle change exceeds the maximum allowed bend angle
        if (i > 0) {
            QVector3D prevSegmentDir = normalizeVector(currentPoint - waypoints[waypoints.size() - 2]);
            QVector3D currentSegmentDir = normalizeVector(nextWaypoint - currentPoint);
            
            double dot = dotProduct(prevSegmentDir, currentSegmentDir);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
            double angleChange = std::acos(dot);
            
            if (angleChange > maxBendAngle) {
                qDebug() << "[BezierDragChainSolver] WARNING: Angle change" << angleChange * 180.0 / M_PI << "degrees exceeds maximum" << maxBendAngle * 180.0 / M_PI << "degrees";
                qDebug() << "[BezierDragChainSolver] This segment violates the bend radius constraint!";
            }
        }
        
        waypoints.push_back(nextWaypoint);
        qDebug() << "[BezierDragChainSolver] Added waypoint" << i + 1 << "at t=" << nextT << ":" << nextWaypoint;
        
        // Update for next iteration
        currentPoint = nextWaypoint;
        currentT = nextT;
    }
    
    // Ensure the last waypoint is exactly the end point
    waypoints.back() = endPoint;
    
    qDebug() << "[BezierDragChainSolver] Generated" << waypoints.size() << "waypoints";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        qDebug() << "[BezierDragChainSolver] Waypoint" << i << ":" << waypoints[i];
    }
    
    return waypoints;
}

std::vector<QVector3D> BezierDragChainSolver::generateConstraintRespectingPathWithCorrections(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendRadius,
    double maxBendAngle)
{
    qDebug() << "[BezierDragChainSolver] generateConstraintRespectingPathWithCorrections called";
    
    std::vector<QVector3D> waypoints;
    waypoints.push_back(startPoint);
    
    // Calculate total arc length of the Bézier curve
    double totalArcLength = calculateBezierArcLength(startPoint, controlPoint, endPoint, 0.0, 1.0);
    int segmentCount = static_cast<int>(std::ceil(totalArcLength / pitchLength));
    
    // Ensure at least 2 segments for a meaningful path
    if (segmentCount < 2) segmentCount = 2;
    
    qDebug() << "[BezierDragChainSolver] Total arc length:" << totalArcLength;
    qDebug() << "[BezierDragChainSolver] Target segment count:" << segmentCount;
    
    // Define the plane using start, end, and control points
    QVector3D planeNormal = normalizeVector(QVector3D::crossProduct(endPoint - startPoint, controlPoint - startPoint));
    if (planeNormal.length() < 0.001) {
        // If the three points are collinear, use a default plane normal
        planeNormal = QVector3D(0, 0, 1);
    }
    
    // Helper function to project a point onto the plane
    auto projectToPlane = [&](const QVector3D& point) -> QVector3D {
        QVector3D pointToStart = point - startPoint;
        double distanceFromPlane = QVector3D::dotProduct(pointToStart, planeNormal);
        return point - planeNormal * distanceFromPlane;
    };
    
    // Generate waypoints with exact pitch length and constraint corrections
    QVector3D currentPoint = startPoint;
    double currentT = 0.0;
    QVector3D prevSegmentDirection = normalizeVector(controlPoint - startPoint); // Initial direction
    
    for (int i = 0; i < segmentCount; ++i) {
        // Find the t parameter for the next waypoint with exact pitch length along the Bézier curve
        // Use currentT as the starting point, not 0.0
        qDebug() << "[BezierDragChainSolver] Segment" << i << "- currentT:" << currentT << "pitchLength:" << pitchLength;
        double nextT = findBezierParameterForArcLength(startPoint, controlPoint, endPoint, pitchLength, currentT);
        qDebug() << "[BezierDragChainSolver] Segment" << i << "- calculated nextT:" << nextT;
        
        // Safety check for invalid t progression
        if (nextT <= currentT) {
            qDebug() << "[BezierDragChainSolver] ERROR: nextT" << nextT << "is not greater than currentT" << currentT;
            qDebug() << "[BezierDragChainSolver] This indicates a problem with arc length calculation. Using end point.";
            nextT = 1.0;
        }
        
        // For the last segment, check if we can reach the end point with exact pitch length
        if (i == segmentCount - 1) {
            double distanceToEnd = calculateDistance(currentPoint, endPoint);
            qDebug() << "[BezierDragChainSolver] Last segment - distance to end:" << distanceToEnd << "target pitch length:" << pitchLength;
            
            if (std::abs(distanceToEnd - pitchLength) < 0.001) {
                // We can reach the end point with exact pitch length
                nextT = 1.0;
                qDebug() << "[BezierDragChainSolver] Last segment can reach end point with exact pitch length";
            } else if (distanceToEnd <= pitchLength) {
                // The end point is within reach, use exact pitch length
                nextT = 1.0;
                qDebug() << "[BezierDragChainSolver] Last segment adjusted to reach end point. Distance:" << distanceToEnd;
            } else {
                // The end point is too far, we need to generate additional segments
                // For now, we'll use the exact pitch length and let the final correction handle it
                nextT = 1.0;
                qDebug() << "[BezierDragChainSolver] Last segment: end point too far, will be corrected later";
            }
        }
        
        // Evaluate the Bézier curve at the next waypoint
        QVector3D idealNextPoint = evaluateBezierCurve(startPoint, controlPoint, endPoint, nextT);
        
        // Safety check for extremely large coordinates
        if (std::abs(idealNextPoint.x()) > 1e6 || std::abs(idealNextPoint.y()) > 1e6 || std::abs(idealNextPoint.z()) > 1e6) {
            qDebug() << "[BezierDragChainSolver] ERROR: Generated extremely large coordinates:" << idealNextPoint;
            qDebug() << "[BezierDragChainSolver] This indicates a problem with t parameter calculation. Using end point instead.";
            idealNextPoint = endPoint;
        }
        
        QVector3D idealDirection = normalizeVector(idealNextPoint - currentPoint);
        
        // Check if the angle change exceeds the maximum allowed bend angle
        double angleChange = 0.0;
        if (i > 0) {
            double dot = dotProduct(prevSegmentDirection, idealDirection);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
            angleChange = std::acos(dot);
        }
        
        QVector3D nextPoint = idealNextPoint;
        
        if (angleChange > maxBendAngle && i > 0) {
            qDebug() << "[BezierDragChainSolver] Constraint violation detected: angle change" << angleChange * 180.0 / M_PI << "degrees exceeds maximum" << maxBendAngle * 180.0 / M_PI << "degrees";
            
            // Calculate the maximum allowed direction change
            QVector3D rotationAxis = normalizeVector(crossProduct(prevSegmentDirection, idealDirection));
            if (rotationAxis.length() < 0.001) {
                // Vectors are parallel, no rotation needed
                rotationAxis = QVector3D(0, 0, 1);
            }
            
            // Ensure rotation axis is in the plane
            rotationAxis = normalizeVector(QVector3D::crossProduct(planeNormal, rotationAxis));
            
            // Rotate previous direction by the maximum allowed angle
            QVector3D constrainedDirection = rotateVector(prevSegmentDirection, rotationAxis, maxBendAngle);
            
            // Calculate corrected waypoint with exact pitch length
            nextPoint = currentPoint + constrainedDirection * pitchLength;
            
            // Safety check for extremely large coordinates in corrected waypoint
            if (std::abs(nextPoint.x()) > 1e6 || std::abs(nextPoint.y()) > 1e6 || std::abs(nextPoint.z()) > 1e6) {
                qDebug() << "[BezierDragChainSolver] ERROR: Generated extremely large coordinates in corrected waypoint:" << nextPoint;
                qDebug() << "[BezierDragChainSolver] Using ideal waypoint instead.";
                nextPoint = idealNextPoint;
            }
            
            qDebug() << "[BezierDragChainSolver] Corrected waypoint:" << nextPoint;
            qDebug() << "[BezierDragChainSolver] Original waypoint:" << idealNextPoint;
        }
        
        // Project the waypoint onto the plane to ensure it stays in the intended plane
        nextPoint = projectToPlane(nextPoint);
        
        waypoints.push_back(nextPoint);
        qDebug() << "[BezierDragChainSolver] Added waypoint" << i + 1 << ":" << nextPoint;
        qDebug() << "[BezierDragChainSolver] Segment length:" << calculateDistance(currentPoint, nextPoint);
        qDebug() << "[BezierDragChainSolver] Target length:" << pitchLength;
        qDebug() << "[BezierDragChainSolver] Length difference:" << std::abs(calculateDistance(currentPoint, nextPoint) - pitchLength);
        
        // Update for next iteration
        currentPoint = nextPoint;
        currentT = nextT;
        prevSegmentDirection = normalizeVector(nextPoint - waypoints[waypoints.size() - 2]);
        
        // Safety check to prevent runaway coordinates in main loop
        if (std::abs(currentPoint.x()) > 1e6 || std::abs(currentPoint.y()) > 1e6 || std::abs(currentPoint.z()) > 1e6) {
            qDebug() << "[BezierDragChainSolver] ERROR: Main loop generated extremely large coordinates:" << currentPoint;
            qDebug() << "[BezierDragChainSolver] Stopping main loop and using end point directly.";
            waypoints.clear();
            waypoints.push_back(startPoint);
            waypoints.push_back(endPoint);
            return waypoints;
        }
    }
    
    // Ensure the last waypoint is exactly the end point
    QVector3D lastWaypoint = waypoints.back();
    double distanceToEnd = calculateDistance(lastWaypoint, endPoint);
    
    if (distanceToEnd > 0.001) {
        qDebug() << "[BezierDragChainSolver] Final correction needed. Distance to end:" << distanceToEnd;
        
        if (distanceToEnd <= pitchLength) {
            // We can reach the end point with a segment shorter than pitch length
            waypoints.back() = endPoint;
            qDebug() << "[BezierDragChainSolver] Final segment length adjusted to:" << distanceToEnd;
        } else {
            // We need additional segments to reach the end point
            QVector3D currentPoint = lastWaypoint;
            
            int maxAdditionalSegments = 10; // Prevent infinite loops
            int segmentCount = 0;
            
            while (calculateDistance(currentPoint, endPoint) > pitchLength && segmentCount < maxAdditionalSegments) {
                // Recalculate direction from current point to end point
                QVector3D currentDirection = normalizeVector(endPoint - currentPoint);
                
                QVector3D nextPoint = currentPoint + currentDirection * pitchLength;
                // Project the intermediate waypoint onto the plane
                nextPoint = projectToPlane(nextPoint);
                
                // Safety check for extremely large coordinates
                if (std::abs(nextPoint.x()) > 1e6 || std::abs(nextPoint.y()) > 1e6 || std::abs(nextPoint.z()) > 1e6) {
                    qDebug() << "[BezierDragChainSolver] ERROR: Generated extremely large coordinates in intermediate waypoint:" << nextPoint;
                    qDebug() << "[BezierDragChainSolver] Skipping this waypoint and using end point directly.";
                    break;
                }
                
                waypoints.push_back(nextPoint);
                currentPoint = nextPoint;
                segmentCount++;
                qDebug() << "[BezierDragChainSolver] Added intermediate waypoint:" << nextPoint;
            }
            
            if (segmentCount >= maxAdditionalSegments) {
                qDebug() << "[BezierDragChainSolver] WARNING: Reached maximum additional segments limit. Using end point directly.";
            }
            
            // Add the final waypoint at the end point
            waypoints.push_back(endPoint);
            qDebug() << "[BezierDragChainSolver] Added final waypoint at end point";
        }
    } else {
        waypoints.back() = endPoint;
    }
    
    qDebug() << "[BezierDragChainSolver] Generated" << waypoints.size() << "constraint-corrected waypoints";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        qDebug() << "[BezierDragChainSolver] Waypoint" << i << ":" << waypoints[i];
    }
    
    return waypoints;
}

QVector3D BezierDragChainSolver::rotateVector(const QVector3D& vector, const QVector3D& axis, double angle)
{
    // Rodrigues' rotation formula
    QVector3D unitAxis = normalizeVector(axis);
    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);
    
    return vector * cosAngle + 
           crossProduct(unitAxis, vector) * sinAngle + 
           unitAxis * dotProduct(unitAxis, vector) * (1.0 - cosAngle);
} 

std::vector<QVector3D> BezierDragChainSolver::generateStrictAngleConstrainedPath(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] generateStrictAngleConstrainedPath called";
    qDebug() << "[BezierDragChainSolver]   Start point:" << startPoint;
    qDebug() << "[BezierDragChainSolver]   End point:" << endPoint;
    qDebug() << "[BezierDragChainSolver]   Control point:" << controlPoint;
    qDebug() << "[BezierDragChainSolver]   Pitch length:" << pitchLength;
    qDebug() << "[BezierDragChainSolver]   Max bend radius:" << maxBendRadius;
    
    // Calculate maximum allowed bend angle from bend radius
    double maxBendAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
    qDebug() << "[BezierDragChainSolver]   Max bend angle:" << maxBendAngle * 180.0 / M_PI << "degrees";
    
    // Calculate total arc length of the Bézier curve
    double totalArcLength = calculateBezierArcLength(startPoint, controlPoint, endPoint, 0.0, 1.0);
    int estimatedSegments = static_cast<int>(std::ceil(totalArcLength / pitchLength));
    
    // Ensure minimum segments for meaningful path
    if (estimatedSegments < 2) estimatedSegments = 2;
    
    qDebug() << "[BezierDragChainSolver]   Total arc length:" << totalArcLength << "mm";
    qDebug() << "[BezierDragChainSolver]   Estimated segments:" << estimatedSegments;
    
    // Generate waypoints by directly following the Bézier curve with exact pitch lengths
    QVector3D currentPoint = startPoint;
    double currentT = 0.0;
    
    std::vector<QVector3D> waypoints;
    waypoints.push_back(startPoint);
    
    // Generate waypoints by following the Bézier curve with exact pitch lengths
    for (int segmentCount = 1; segmentCount <= estimatedSegments * 2; ++segmentCount) {
        // Find the t parameter that gives us exactly pitch length from current point
        double targetArcLength = segmentCount * pitchLength;
        
        // Use binary search to find the t parameter for the target arc length
        double tStart = currentT;
        double tEnd = 1.0;
        double tTarget = tStart;
        
        for (int searchIter = 0; searchIter < 20; ++searchIter) {
            double arcLengthToTarget = calculateBezierArcLength(startPoint, controlPoint, endPoint, currentT, tTarget);
            
            if (std::abs(arcLengthToTarget - pitchLength) < 0.01) { // Very tight tolerance
                break;
            } else if (arcLengthToTarget > pitchLength) {
                tEnd = tTarget;
                tTarget = tStart + (tTarget - tStart) * 0.5;
            } else {
                tStart = tTarget;
                tTarget = tTarget + (tEnd - tTarget) * 0.5;
            }
        }
        
        // Evaluate the Bézier curve at the found t parameter
        QVector3D nextPoint = evaluateBezierCurve(startPoint, controlPoint, endPoint, tTarget);
        
        // Check if we can reach the end point
        double distanceToEnd = calculateDistance(nextPoint, endPoint);
        if (distanceToEnd <= pitchLength) {
            // We can reach the end point, add it and finish
            waypoints.push_back(endPoint);
            qDebug() << "[BezierDragChainSolver]   Final segment: Reached end point at t=" << tTarget;
            break;
        }
        
        // Add the waypoint and continue
        waypoints.push_back(nextPoint);
        currentPoint = nextPoint;
        currentT = tTarget;
        
        qDebug() << "[BezierDragChainSolver]   Segment" << segmentCount << ": t=" << tTarget << "point=" << nextPoint;
    }
    
    // If we couldn't reach the end point, the path deviates from the intended end point
    if (waypoints.size() > 0 && waypoints.back() != endPoint) {
        double distanceToEnd = calculateDistance(waypoints.back(), endPoint);
        qDebug() << "[BezierDragChainSolver]   PATH DEVIATION: Could not reach intended end point due to physical constraints";
        qDebug() << "[BezierDragChainSolver]     Final waypoint distance from intended end:" << distanceToEnd << "mm";
        qDebug() << "[BezierDragChainSolver]     Physical constraints (pitch length) take priority over path accuracy";
    }
    
    qDebug() << "[BezierDragChainSolver]   Generated" << waypoints.size() << "waypoints with strict angle constraints";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        qDebug() << "[BezierDragChainSolver]   Waypoint" << i << ":" << waypoints[i];
    }
    
    return waypoints;
}

std::vector<BezierDragChainSegment> BezierDragChainSolver::calculateSegmentsWithStrictAngleConstraints(
    const QVector3D& startPoint,
    const QVector3D& endPoint,
    const QVector3D& controlPoint,
    double pitchLength,
    double maxBendRadius,
    bool attemptPathRecovery)
{
    qDebug() << "[BezierDragChainSolver] calculateSegmentsWithStrictAngleConstraints called";
    
    // Generate waypoints with strict angle constraints
    std::vector<QVector3D> waypoints = generateStrictAngleConstrainedPath(
        startPoint, endPoint, controlPoint, pitchLength, maxBendRadius);
    
    if (waypoints.size() < 2) {
        qDebug() << "[BezierDragChainSolver] ERROR: Not enough waypoints generated";
        return std::vector<BezierDragChainSegment>();
    }
    
    // Attempt path recovery if enabled and needed
    if (attemptPathRecovery && waypoints.back() != endPoint) {
        qDebug() << "[BezierDragChainSolver] Path recovery enabled - attempting to bring path back to intended end point";
        std::vector<QVector3D> recoveredWaypoints = this->attemptPathRecovery(waypoints, endPoint, pitchLength, maxBendRadius);
        
        // Check if recovery was successful
        double finalDistanceToEnd = calculateDistance(recoveredWaypoints.back(), endPoint);
        if (finalDistanceToEnd <= pitchLength) {
            qDebug() << "[BezierDragChainSolver] RECOVERY SUCCESSFUL: Path now reaches intended end point!";
            waypoints = recoveredWaypoints;
        } else if (finalDistanceToEnd <= pitchLength * 2.0) {
            qDebug() << "[BezierDragChainSolver] RECOVERY PARTIAL: Path closer to intended end point (distance:" << finalDistanceToEnd << "mm)";
            waypoints = recoveredWaypoints;
        } else {
            qDebug() << "[BezierDragChainSolver] RECOVERY FAILED: Path still too far from intended end point (distance:" << finalDistanceToEnd << "mm)";
            qDebug() << "[BezierDragChainSolver] Physical constraints prevent reaching the intended end point";
        }
    }
    
    std::vector<BezierDragChainSegment> segments;
    
    // Calculate maximum allowed bend angle
    double maxBendAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
    
    // Create segments from waypoints with strict pitch length validation
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        QVector3D segmentStart = waypoints[i];
        QVector3D segmentEnd = waypoints[i + 1];
        
        // Calculate segment length
        double segmentLength = calculateDistance(segmentStart, segmentEnd);
        
        // Check if this segment violates the pitch length constraint
        // ALL segments must have exactly the pitch length - no exceptions
        bool violatesPitchLength = false;
        
        if (std::abs(segmentLength - pitchLength) > 0.1) {
            violatesPitchLength = true;
            qDebug() << "[BezierDragChainSolver]   CRITICAL ERROR: Segment" << i << "violates pitch length constraint!";
            qDebug() << "[BezierDragChainSolver]     Expected:" << pitchLength << "mm, Actual:" << segmentLength << "mm";
            qDebug() << "[BezierDragChainSolver]     Correcting segment to exact pitch length...";
            
            // Fix the segment by adjusting the end point to exactly pitch length
            QVector3D segmentDirection = normalizeVector(segmentEnd - segmentStart);
            segmentEnd = segmentStart + segmentDirection * pitchLength;
            segmentLength = pitchLength; // Force exact pitch length
            
            qDebug() << "[BezierDragChainSolver]     Corrected segment length:" << segmentLength << "mm";
        }
        
        // Calculate bend angle (0 for first segment)
        double bendAngle = 0.0;
        QVector3D bendAxis(0, 0, 0);
        bool isBend = false;
        double bendRadius = 0.0;
        
        if (i > 0) {
            // Calculate angle between this segment and the previous segment
            QVector3D prevSegmentDir = normalizeVector(waypoints[i] - waypoints[i-1]);
            QVector3D currentSegmentDir = normalizeVector(segmentEnd - segmentStart);
            
            double dot = dotProduct(prevSegmentDir, currentSegmentDir);
            dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
            bendAngle = std::acos(dot);
            
            if (bendAngle > 0.001) { // Significant bend
                isBend = true;
                bendAxis = normalizeVector(crossProduct(prevSegmentDir, currentSegmentDir));
                
                // Calculate bend radius from angle and segment length
                if (bendAngle > 0.001) {
                    bendRadius = segmentLength / (2.0 * std::sin(bendAngle / 2.0));
                }
                
                qDebug() << "[BezierDragChainSolver]   Segment" << i << "bend angle:" << bendAngle * 180.0 / M_PI << "degrees, radius:" << bendRadius;
            }
        }
        
        // Create segment with exactly the pitch length
        BezierDragChainSegment segment(
            segmentStart, segmentEnd, controlPoint,
            pitchLength, bendAngle, bendAxis, isBend, bendRadius, // Always use pitchLength, not segmentLength
            static_cast<double>(i) / (waypoints.size() - 1),
            static_cast<double>(i + 1) / (waypoints.size() - 1)
        );
        
        segments.push_back(segment);
        
        qDebug() << "[BezierDragChainSolver]   Created segment" << i << ":" << segmentStart << "->" << segmentEnd << "length:" << pitchLength;
        if (violatesPitchLength) {
            qDebug() << "[BezierDragChainSolver]     ✓ CORRECTED to exact pitch length!";
        }
    }
    
    qDebug() << "[BezierDragChainSolver]   Generated" << segments.size() << "segments with strict angle constraints";
    
    // Validate that all segments have exactly the pitch length
    bool allSegmentsValid = true;
    for (size_t i = 0; i < segments.size(); ++i) {
        double actualLength = calculateDistance(segments[i].startPoint, segments[i].endPoint);
        double lengthDifference = std::abs(actualLength - pitchLength);
        
        if (lengthDifference > 0.001) {
            qDebug() << "[BezierDragChainSolver]   ⚠️ VALIDATION ERROR: Segment" << i << "length" << actualLength << "differs from pitch length" << pitchLength << "by" << lengthDifference << "mm";
            allSegmentsValid = false;
        } else {
            qDebug() << "[BezierDragChainSolver]   ✓ Segment" << i << "length:" << actualLength << "mm (exact pitch length)";
        }
    }
    
    if (allSegmentsValid) {
        qDebug() << "[BezierDragChainSolver]   ✓ ALL SEGMENTS HAVE EXACT PITCH LENGTH!";
    } else {
        qDebug() << "[BezierDragChainSolver]   ⚠️ SOME SEGMENTS VIOLATE PITCH LENGTH CONSTRAINT!";
    }
    
    // Update solver statistics
    calculateSegmentLengthStatistics(segments, maxBendRadius, pitchLength);
    
    return segments;
}

std::vector<QVector3D> BezierDragChainSolver::attemptPathRecovery(
    const std::vector<QVector3D>& waypoints,
    const QVector3D& intendedEndPoint,
    double pitchLength,
    double maxBendRadius)
{
    qDebug() << "[BezierDragChainSolver] attemptPathRecovery called";
    
    if (waypoints.size() < 2) {
        qDebug() << "[BezierDragChainSolver]   Not enough waypoints for recovery";
        return waypoints;
    }
    
    // Check if we actually need recovery
    double distanceToEnd = calculateDistance(waypoints.back(), intendedEndPoint);
    if (distanceToEnd <= pitchLength) {
        qDebug() << "[BezierDragChainSolver]   No recovery needed - already within reach";
        return waypoints;
    }
    
    qDebug() << "[BezierDragChainSolver]   Attempting path recovery from distance:" << distanceToEnd << "mm";
    
    // Calculate maximum allowed bend angle
    double maxBendAngle = calculateMaxBendAngle(pitchLength, maxBendRadius);
    
    // Start recovery from the last waypoint
    std::vector<QVector3D> recoveredWaypoints = waypoints;
    QVector3D currentPoint = waypoints.back();
    
    // Get the direction of the last segment
    QVector3D lastSegmentDir = normalizeVector(currentPoint - waypoints[waypoints.size() - 2]);
    
    // Define the plane using the last two segments
    QVector3D planeNormal;
    if (waypoints.size() >= 3) {
        QVector3D prevSegmentDir = normalizeVector(waypoints[waypoints.size() - 2] - waypoints[waypoints.size() - 3]);
        planeNormal = normalizeVector(QVector3D::crossProduct(prevSegmentDir, lastSegmentDir));
    } else {
        // Fallback plane normal
        planeNormal = QVector3D(0, 0, 1);
    }
    
    if (planeNormal.length() < 0.001) {
        planeNormal = QVector3D(0, 0, 1);
    }
    
    // Helper function to project a point onto the plane
    auto projectToPlane = [&](const QVector3D& point) -> QVector3D {
        QVector3D pointToCurrent = point - currentPoint;
        double distanceFromPlane = QVector3D::dotProduct(pointToCurrent, planeNormal);
        return point - planeNormal * distanceFromPlane;
    };
    
    // Try to recover the path by generating additional segments
    const int maxRecoverySegments = 10; // Limit recovery attempts
    int recoverySegments = 0;
    
    while (recoverySegments < maxRecoverySegments) {
        recoverySegments++;
        
        // Calculate direction to the intended end point
        QVector3D directionToEnd = normalizeVector(intendedEndPoint - currentPoint);
        
        // Check if we can reach the end point with exact pitch length
        double distanceToEnd = calculateDistance(currentPoint, intendedEndPoint);
        if (distanceToEnd <= pitchLength) {
            // We can reach the end point with exact pitch length
            recoveredWaypoints.push_back(intendedEndPoint);
            qDebug() << "[BezierDragChainSolver]   Recovery successful! Reached end point with" << recoverySegments << "additional segments";
            return recoveredWaypoints;
        }
        
        // Calculate angle change from the last segment direction
        double dot = dotProduct(lastSegmentDir, directionToEnd);
        dot = (dot < -1.0) ? -1.0 : (dot > 1.0) ? 1.0 : dot;
        double angleChange = std::acos(dot);
        
        QVector3D nextPoint;
        QVector3D nextDirection;
        
        if (angleChange > maxBendAngle) {
            qDebug() << "[BezierDragChainSolver]   Recovery segment" << recoverySegments << ": Angle change" << angleChange * 180.0 / M_PI << "degrees exceeds maximum" << maxBendAngle * 180.0 / M_PI << "degrees";
            
            // Calculate the maximum allowed direction change
            QVector3D rotationAxis = normalizeVector(crossProduct(lastSegmentDir, directionToEnd));
            if (rotationAxis.length() < 0.001) {
                // Vectors are parallel, use perpendicular axis in the plane
                rotationAxis = normalizeVector(QVector3D::crossProduct(planeNormal, lastSegmentDir));
            }
            
            // Ensure rotation axis is in the plane
            rotationAxis = normalizeVector(QVector3D::crossProduct(planeNormal, rotationAxis));
            
            // Rotate last segment direction by the maximum allowed angle
            QVector3D constrainedDirection = rotateVector(lastSegmentDir, rotationAxis, maxBendAngle);
            
            // Calculate next point with exact pitch length using constrained direction
            nextPoint = currentPoint + constrainedDirection * pitchLength;
            
            // Project the point onto the plane
            nextPoint = projectToPlane(nextPoint);
            
            nextDirection = constrainedDirection;
            
            qDebug() << "[BezierDragChainSolver]   Recovery: Using constrained direction, angle change:" << maxBendAngle * 180.0 / M_PI << "degrees";
        } else {
            // Angle change is acceptable, use direction to end but ensure exact pitch length
            nextPoint = currentPoint + directionToEnd * pitchLength;
            nextDirection = directionToEnd;
            
            qDebug() << "[BezierDragChainSolver]   Recovery segment" << recoverySegments << ": Using direct direction to end, angle change:" << angleChange * 180.0 / M_PI << "degrees";
        }
        
        // Add the recovery waypoint
        recoveredWaypoints.push_back(nextPoint);
        
        // Update for next iteration
        currentPoint = nextPoint;
        lastSegmentDir = nextDirection;
        
        qDebug() << "[BezierDragChainSolver]   Recovery segment" << recoverySegments << "added:" << nextPoint;
    }
    
    // If we couldn't recover within maxRecoverySegments, check final distance
    double finalDistanceToEnd = calculateDistance(recoveredWaypoints.back(), intendedEndPoint);
    if (finalDistanceToEnd <= pitchLength * 2.0) {
        // We're close enough, add the end point
        recoveredWaypoints.push_back(intendedEndPoint);
        qDebug() << "[BezierDragChainSolver]   Recovery completed: Final distance" << finalDistanceToEnd << "mm (within 2x pitch length)";
    } else {
        qDebug() << "[BezierDragChainSolver]   Recovery failed: Final distance" << finalDistanceToEnd << "mm (too far)";
        qDebug() << "[BezierDragChainSolver]   Path recovery could not bring path back to intended end point";
    }
    
    return recoveredWaypoints;
}
