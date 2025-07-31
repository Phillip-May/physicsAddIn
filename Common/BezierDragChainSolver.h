#pragma once

#include <QVector3D>
#include <QVector4D>
#include <vector>
#include <memory>
#include <QDebug>
#include <cmath>
#include <functional>

// Forward declarations
class CadOpenGLWidget;

// Define ConnectionPathSegment structure to avoid circular dependency
struct ConnectionPathSegment {
    QVector3D start;
    QVector3D end;
    QVector4D color;
    float width;
    bool isBend;
    
    ConnectionPathSegment(const QVector3D& s, const QVector3D& e, const QVector4D& c = QVector4D(1, 0, 0, 1), float w = 2.0f, bool bend = false)
        : start(s), end(e), color(c), width(w), isBend(bend) {}
};

// Bézier curve-based drag chain segment
struct BezierDragChainSegment {
    QVector3D startPoint;
    QVector3D endPoint;
    QVector3D controlPoint;      // Control point for quadratic Bézier
    double length;               // Arc length of this segment
    double bendAngle;            // Angle from previous segment (radians)
    QVector3D bendAxis;          // Rotation axis for bend
    bool isBend;
    double bendRadius;
    double tStart;               // Parameter t at start of segment (0-1)
    double tEnd;                 // Parameter t at end of segment (0-1)
    
    BezierDragChainSegment(const QVector3D& start, const QVector3D& end, const QVector3D& control,
                           double segLength, double angle = 0.0, const QVector3D& axis = QVector3D(0,0,0), 
                           bool bend = false, double radius = 0.0, double tS = 0.0, double tE = 1.0)
        : startPoint(start), endPoint(end), controlPoint(control), length(segLength), 
          bendAngle(angle), bendAxis(axis), isBend(bend), bendRadius(radius), tStart(tS), tEnd(tE) {}
};

// Bézier curve constraint structure
struct BezierDragChainConstraint {
    enum Type {
        StartPoint,          // Must start at specific point
        EndPoint,           // Must end at specific point  
        StartDirection,      // First segment must have specific direction
        EndDirection,        // Last segment must have specific direction
        MaxBendRadius,       // Bend radius must be <= maximum
        FixedPitchLength,    // All segments must have exact pitch length
        ControlPointLimit,   // Control point position limits
        Smoothness          // Curve smoothness constraint
    };
    
    Type type;
    QVector3D value;        // Constraint value (point, direction, etc.)
    double tolerance;        // Constraint tolerance
    
    BezierDragChainConstraint(Type t, const QVector3D& val, double tol = 0.1)
        : type(t), value(val), tolerance(tol) {}
};

class BezierDragChainSolver {
public:
    BezierDragChainSolver();
    ~BezierDragChainSolver();
    
    // Main solving function using quadratic Bézier curves
    std::vector<BezierDragChainSegment> solveDragChainPath(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& startDirection,
        const QVector3D& endDirection,
        double pitchLength,
        double maxBendRadius,
        int segmentCount,
        bool startLocked,
        bool endLocked,
        const QVector3D& startPlaneNormal = QVector3D(0, 0, 1),
        const QVector3D& endPlaneNormal = QVector3D(0, 0, 1)
    );
    
    // Simplified solving function - no iterations, just mathematical calculation
    std::vector<BezierDragChainSegment> solveDragChainPathSimple(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,  // User-defined control point
        double pitchLength,
        double maxBendRadius
    );
    
    // Multi-control point solving function
    std::vector<BezierDragChainSegment> solveDragChainPathWithMultipleControlPoints(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const std::vector<QVector3D>& controlPoints,  // Multiple user-defined control points
        double pitchLength,
        double maxBendRadius
    );
    
    // Constraint-based solving function that respects physical constraints
    std::vector<BezierDragChainSegment> solveDragChainPathWithConstraints(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendRadius
    );
    
    // Generate constraint-respecting path
    std::vector<BezierDragChainSegment> generateConstraintRespectingPath(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendRadius,
        int targetSegmentCount
    );
    
    // Generate waypoints that respect bend angle constraints
    std::vector<QVector3D> generateConstraintRespectingWaypoints(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendAngle
    );
    
    // Generate constraint-respecting path with corrections for violations
    std::vector<QVector3D> generateConstraintRespectingPathWithCorrections(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendRadius,
        double maxBendAngle
    );
    
    // NEW: Generate path with strict angle enforcement - deviates from ideal path when needed
    std::vector<QVector3D> generateStrictAngleConstrainedPath(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendRadius
    );
    
    // NEW: Calculate segments with enforced angle constraints
    std::vector<BezierDragChainSegment> calculateSegmentsWithStrictAngleConstraints(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& controlPoint,
        double pitchLength,
        double maxBendRadius,
        bool attemptPathRecovery = true
    );
    
    // NEW: Attempt to recover path back to intended end point after deviation
    std::vector<QVector3D> attemptPathRecovery(
        const std::vector<QVector3D>& waypoints,
        const QVector3D& intendedEndPoint,
        double pitchLength,
        double maxBendRadius
    );
    
    // Rotate a vector around an axis by a given angle
    QVector3D rotateVector(const QVector3D& vector, const QVector3D& axis, double angle);
    
    // Calculate segments along a user-defined Bézier curve with fixed pitch length
    std::vector<BezierDragChainSegment> calculateSegmentsAlongCurve(
        const QVector3D& p0,  // Start point
        const QVector3D& p1,  // Control point (user-defined)
        const QVector3D& p2,  // End point
        double pitchLength,
        double maxBendRadius
    );
    
    // Calculate angle between two segments
    double calculateSegmentAngle(const QVector3D& segment1Start, const QVector3D& segment1End,
                               const QVector3D& segment2Start, const QVector3D& segment2End);
    
    // Calculate the total angle at a waypoint where multiple segments connect
    double calculateWaypointAngle(const std::vector<BezierDragChainSegment>& segments, size_t waypointIndex);
    
    // Calculate optimal segment count based on geometry and constraints
    int calculateOptimalSegmentCount(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& startDirection,
        const QVector3D& endDirection,
        double pitchLength,
        double maxBendRadius,
        bool startLocked,
        bool endLocked
    );
    
    // Calculate minimum required segments for a given connection
    int calculateMinimumSegments(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        double pitchLength,
        double maxBendRadius
    );
    
    // Calculate maximum practical segments for a given connection
    int calculateMaximumSegments(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        double pitchLength,
        double maxBendRadius
    );
    
    // Convert solved path to visualization segments
    std::vector<ConnectionPathSegment> convertToVisualizationSegments(
        const std::vector<BezierDragChainSegment>& segments,
        const QVector4D& mainColor = QVector4D(1, 0, 0, 0.8f),
        const QVector4D& indicatorColor = QVector4D(0, 1, 0, 1.0f)
    );
    
    // Enhanced visualization with better alternating colors and indicators
    std::vector<ConnectionPathSegment> convertToEnhancedVisualization(
        const std::vector<BezierDragChainSegment>& segments,
        const QVector4D& primaryColor = QVector4D(1, 0, 0, 0.8f),
        const QVector4D& secondaryColor = QVector4D(0, 0, 1, 0.8f),
        const QVector4D& tertiaryColor = QVector4D(0, 1, 0, 0.8f),
        const QVector4D& indicatorColor = QVector4D(1, 1, 0, 1.0f)
    );
    
    // Get path statistics from solved segments
    struct PathStatistics {
        double totalLength;
        int totalSegments;
        int straightSegments;
        int bendSegments;
        double averageSegmentLength;
        double maxSegmentLength;
        double minSegmentLength;
        bool isValid;
        
        PathStatistics() : totalLength(0.0), totalSegments(0), straightSegments(0), bendSegments(0),
                          averageSegmentLength(0.0), maxSegmentLength(0.0), minSegmentLength(0.0), isValid(false) {}
    };
    
    PathStatistics calculatePathStatistics(const std::vector<BezierDragChainSegment>& segments);
    
    // Solver statistics structure
    struct SolverStatistics {
        int iterationsUsed;
        double finalDistanceToTarget;
        bool converged;
        double computationTime;
        int controlPointsOptimized;
        double minSegmentLength;
        double maxSegmentLength;
        double averageSegmentLength;
        double minSegmentAngle;
        double maxSegmentAngle;
        double averageSegmentAngle;
        double totalCableLength;
        int totalSegments;
        
        SolverStatistics() : iterationsUsed(0), finalDistanceToTarget(0.0), converged(false), 
                           computationTime(0.0), controlPointsOptimized(0),
                           minSegmentLength(0.0), maxSegmentLength(0.0), averageSegmentLength(0.0),
                           minSegmentAngle(0.0), maxSegmentAngle(0.0), averageSegmentAngle(0.0),
                           totalCableLength(0.0), totalSegments(0) {}
        
        // Copy constructor
        SolverStatistics(const SolverStatistics& other) 
            : iterationsUsed(other.iterationsUsed), finalDistanceToTarget(other.finalDistanceToTarget),
              converged(other.converged), computationTime(other.computationTime),
              controlPointsOptimized(other.controlPointsOptimized),
              minSegmentLength(other.minSegmentLength), maxSegmentLength(other.maxSegmentLength),
              averageSegmentLength(other.averageSegmentLength),
              minSegmentAngle(other.minSegmentAngle), maxSegmentAngle(other.maxSegmentAngle),
              averageSegmentAngle(other.averageSegmentAngle),
              totalCableLength(other.totalCableLength), totalSegments(other.totalSegments) {}
        
        // Assignment operator
        SolverStatistics& operator=(const SolverStatistics& other) {
            if (this != &other) {
                iterationsUsed = other.iterationsUsed;
                finalDistanceToTarget = other.finalDistanceToTarget;
                converged = other.converged;
                computationTime = other.computationTime;
                controlPointsOptimized = other.controlPointsOptimized;
                minSegmentLength = other.minSegmentLength;
                maxSegmentLength = other.maxSegmentLength;
                averageSegmentLength = other.averageSegmentLength;
                minSegmentAngle = other.minSegmentAngle;
                maxSegmentAngle = other.maxSegmentAngle;
                averageSegmentAngle = other.averageSegmentAngle;
                totalCableLength = other.totalCableLength;
                totalSegments = other.totalSegments;
            }
            return *this;
        }
    };
    
    SolverStatistics getLastSolverStatistics() const { 
        qDebug() << "[BezierDragChainSolver] getLastSolverStatistics() called";
        qDebug() << "[BezierDragChainSolver]   Current stats - Distance to target:" << m_lastSolverStatistics.finalDistanceToTarget << "mm";
        qDebug() << "[BezierDragChainSolver]   Current stats - Converged:" << m_lastSolverStatistics.converged;
        qDebug() << "[BezierDragChainSolver]   Current stats - Iterations:" << m_lastSolverStatistics.iterationsUsed;
        qDebug() << "[BezierDragChainSolver]   Current stats - Min segment length:" << m_lastSolverStatistics.minSegmentLength << "mm";
        qDebug() << "[BezierDragChainSolver]   Current stats - Max segment length:" << m_lastSolverStatistics.maxSegmentLength << "mm";
        qDebug() << "[BezierDragChainSolver]   Current stats - Avg segment length:" << m_lastSolverStatistics.averageSegmentLength << "mm";
        SolverStatistics stats = m_lastSolverStatistics;
        return stats;
    }
    
    // Get real-time statistics during solving
    SolverStatistics getCurrentSolverStatistics() const { 
        return m_lastSolverStatistics;
    }
    
    // Update solver statistics in real-time
    void updateSolverStatistics(double distanceToTarget, bool converged, int additionalIterations);
    
    // Calculate segment length statistics from current curve
    void calculateSegmentLengthStatistics(const std::vector<BezierDragChainSegment>& segments, double maxBendRadius = 18.0, double pitchLength = 18.0);
    
    // Reset solver statistics
    void resetSolverStatistics() { m_lastSolverStatistics = SolverStatistics(); }
    
    // Set callback function for real-time statistics updates
    void setStatisticsCallback(std::function<void(const SolverStatistics&)> callback) {
        m_statisticsCallback = callback;
    }
    
    // Solver parameters
    void setMaxIterations(int iterations) { m_maxIterations = iterations; }
    void setConvergenceTolerance(double tolerance) { m_convergenceTolerance = tolerance; }
    void setBendRadiusTolerance(double tolerance) { m_bendRadiusTolerance = tolerance; }
    void setOptimizationStep(double step) { m_optimizationStep = step; }
    void setCurveSmoothness(double smoothness) { m_curveSmoothness = smoothness; }
    
    // Get solver parameters
    int getMaxIterations() const { return m_maxIterations; }
    double getConvergenceTolerance() const { return m_convergenceTolerance; }
    double getBendRadiusTolerance() const { return m_bendRadiusTolerance; }
    double getOptimizationStep() const { return m_optimizationStep; }
    double getCurveSmoothness() const { return m_curveSmoothness; }

private:
    // Core Bézier curve solving functions
    bool createQuadraticBezierCurve(
        const QVector3D& startPoint,
        const QVector3D& endPoint,
        const QVector3D& startDirection,
        const QVector3D& endDirection,
        double pitchLength,
        int segmentCount,
        double maxBendRadius
    );
    
    bool optimizeControlPoints(
        const QVector3D& targetEndPoint,
        const QVector3D& targetEndDirection,
        bool startLocked,
        bool endLocked,
        double maxBendRadius,
        double pitchLength,
        int segmentCount
    );
    
    // Bézier curve mathematical functions
    QVector3D evaluateBezierCurve(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double t);
    QVector3D evaluateBezierDerivative(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double t);
    double calculateBezierArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double tStart = 0.0, double tEnd = 1.0);
    double findBezierParameterForArcLength(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, double targetLength, double tStart = 0.0);
    
    // Segment generation from Bézier curve
    std::vector<BezierDragChainSegment> generateSegmentsFromBezier(
        const QVector3D& p0, const QVector3D& p1, const QVector3D& p2,
        double pitchLength, int segmentCount, double maxBendRadius
    );
    
    // Constraint solving with Bézier curves
    bool applyStartPointConstraint(const QVector3D& startPoint);
    bool applyEndPointConstraint(const QVector3D& endPoint);
    bool applyDirectionConstraints(const QVector3D& startDirection, const QVector3D& endDirection);
    bool applyBendRadiusConstraints(double maxBendRadius, double pitchLength);
    bool applyPitchLengthConstraints(double pitchLength);
    bool applySmoothnessConstraints();
    
    
    // Control point optimization
    QVector3D optimizeControlPointForConstraints(
        const QVector3D& startPoint, const QVector3D& endPoint,
        const QVector3D& startDirection, const QVector3D& endDirection,
        double maxBendRadius, double pitchLength, int segmentCount
    );
    
    // Geometric calculations
    double calculateBendRadius(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2);
    double calculateMaxBendAngle(double segmentLength, double maxBendRadius);
    QVector3D calculatePerpendicularAxis(const QVector3D& direction);
    QVector3D calculateRotationBetweenVectors(const QVector3D& from, const QVector3D& to);
    
    // Visualization helpers
    std::vector<QVector3D> generateBezierPoints(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, int numPoints = 20);
    QVector3D calculateBendCenter(const QVector3D& start, const QVector3D& end, const QVector3D& direction, double radius);
    
    // Constraint validation
    bool validateBendRadiusConstraints(const std::vector<BezierDragChainSegment>& segments, double maxBendRadius);
    bool validateDirectionConstraints(const std::vector<BezierDragChainSegment>& segments, const QVector3D& startDirection, const QVector3D& endDirection);
    bool validatePitchLengthConstraints(const std::vector<BezierDragChainSegment>& segments, double pitchLength);
    
    // Convergence checking
    bool checkConvergence(const QVector3D& targetEndPoint, double tolerance);
    
    // Utility functions
    double calculateDistance(const QVector3D& p1, const QVector3D& p2);
    QVector3D normalizeVector(const QVector3D& vec);
    double dotProduct(const QVector3D& v1, const QVector3D& v2);
    QVector3D crossProduct(const QVector3D& v1, const QVector3D& v2);
    double safeClamp(double value, double min, double max);

private:
    int m_maxIterations;
    double m_convergenceTolerance;
    double m_bendRadiusTolerance;
    double m_optimizationStep;
    double m_curveSmoothness;
    
    // Current Bézier curve parameters
    QVector3D m_currentStartPoint;
    QVector3D m_currentEndPoint;
    QVector3D m_currentControlPoint;
    std::vector<BezierDragChainSegment> m_currentSegments;
    
    // Solver statistics
    SolverStatistics m_lastSolverStatistics;
    
    // Callback function for real-time statistics updates
    std::function<void(const SolverStatistics&)> m_statisticsCallback;
}; 
